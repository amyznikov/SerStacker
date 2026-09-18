/*
 * c_phase_correlate.cc
 *
 *  Created on: Sep 7, 2026
 *      Author: amyznikov
 */
#include "c_phase_correlate.h"
#include <opencv2/geometry.hpp>
#include <core/proc/run-loop.h>
#include <core/proc/fft.h>
#include <core/debug.h>

static int getOptimalFFTSizeDown(int size)
{
  int sopt = cv::getOptimalDFTSize(size);
  while ( sopt > 0 && sopt > size ) {
    sopt = cv::getOptimalDFTSize(--size);
  }
  return sopt;
}

static void packScaledImageForPhaseCorreation(cv::InputArray srcImage, cv::InputArray srcMask,
    cv::Mat1f & outImage, cv::Mat1b & outMask,
    const cv::Size & fftSize,
    cv::Size & outputValidSize,
    cv::Point & cropOffset)
{
  cv::Mat smallImage, smallMask;

  if (srcImage.cols() <= fftSize.width && srcImage.rows() <= fftSize.height) {
    cropOffset = cv::Point(0, 0);
    smallImage = srcImage.getMat();
    smallMask = srcMask.getMat();
    outputValidSize = smallImage.size();
  }
  else {
    const int cropW = std::min(srcImage.cols(), fftSize.width);
    const int cropH = std::min(srcImage.rows(), fftSize.height);
    cropOffset.x = (srcImage.cols() - cropW) / 2;
    cropOffset.y = (srcImage.rows() - cropH) / 2;
    const cv::Rect roi(cropOffset.x, cropOffset.y, cropW, cropH);
    smallImage = srcImage.getMat()(roi);
    if (!srcMask.empty()) {
      smallMask = srcMask.getMat()(roi);
    }
    outputValidSize = cv::Size(cropW, cropH);
  }

  const int bottom = fftSize.height - smallImage.rows;
  const int right = fftSize.width - smallImage.cols;
  if (bottom < 1 && right < 1 ) {
    if ( smallImage.depth() == CV_32F ) {
      outImage = std::move(smallImage);
    }
    else {
      smallImage.convertTo(outImage, CV_32F);
    }
    if ( smallMask.empty() ) {
      outMask = cv::Mat1b(fftSize, 255);
    }
    else if ( smallMask.depth() == CV_8U ) {
      outMask = std::move(smallMask);
    }
    else {
      cv::compare(smallMask, 0, outMask, cv::CMP_GT);
    }
  }
  else {
    const cv::Rect validROI(0, 0, outputValidSize.width, outputValidSize.height);

    outImage.create(fftSize), outImage.setTo(0);
    outMask.create(fftSize), outMask.setTo(0);

    if ( smallImage.depth() == CV_32F ) {
      smallImage.copyTo(outImage(validROI));
    }
    else {
      smallImage.convertTo(outImage(validROI), CV_32F);
    }

    if ( smallMask.empty() ) {
      outMask(validROI).setTo(255);
    }
    else if ( smallMask.depth() == CV_8U ) {
      smallMask.copyTo(outMask(validROI));
    }
    else {
      cv::compare(smallMask, 0, outMask(validROI), cv::CMP_GT);
    }
  }
}

cv::Size c_phase_correlate::computeFFTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor)
{
  const int downscaledW = getOptimalFFTSizeDown(cvRound(expectedFrameSize.width / downscaleFactor));
  const int downscaledH = getOptimalFFTSizeDown(cvRound(expectedFrameSize.height / downscaleFactor));
  return cv::Size(std::max(4,downscaledW), std::max(4,downscaledH));
}


bool c_phase_correlate::setup(const cv::Size & expectedFrameSize, c_phase_correlate_options & opts)
{
  if( expectedFrameSize.empty() ) {
    CF_ERROR("c_phase_correlate: BAD expectedFrameSize specified : %dx%d",
        expectedFrameSize.width, expectedFrameSize.height);
    return false;
  }

  if( (_downscale_factor = opts.downscale_factor) < 1 ) {
    CF_ERROR("c_phase_correlate: BAD downscale_factor specified : %g. Must be >= 1",
        opts.downscale_factor);
    return false;
  }

  _fftSize = computeFFTPackSize(expectedFrameSize, _downscale_factor);
  _expectedFrameSize = expectedFrameSize;
  _gsigma = opts.gsigma;
  _csigma = opts.csigma;
  _calpha = opts.calpha;

  generateBandpassFilter();

  _initialized = true;
  return true;
}

void c_phase_correlate::release()
{
  _fftSize = cv::Size(0, 0);
  _scaledCurrentImage.release();
  _scaledReferenceImage.release();
  _scaledCurrentMask.release();
  _scaledReferenceMask.release();
  _currentSpectrum.release();
  _referenceSpectrum.release();
  _crossSpectrum.release();
  _correlationMap.release();
//  _distmap.release();
  _initialized = false;
}

#if 0
// --- ПРАВИЛЬНАЯ ГЕНЕРАЦИЯ ФИЛЬТРА ПО ТЗ ---
_bandpassFilter.create(_fftSize);

const uint8_t * filter_base = _bandpassFilter.ptr();
const size_t filter_stride = _bandpassFilter.step;

const int cols = _fftSize.width;
const int rows = _fftSize.height;

const float inv_cols = float(1.0 / cols);
const float inv_rows = float(1.0 / rows);
const float lambda2 = float(0.5 * CV_PI * CV_PI * _gsigma * _gsigma);

parallel_for(0, rows, [=](const auto & range) {
  for( int y = rbegin(range); y < rend(range); ++y ) {
    float * __restrict fltp = (float * )(filter_base + y * filter_stride);

    const int fy = (y > rows / 2) ? (rows - y) : y;
    const float v = fy * inv_rows;
    const float v2 = v * v;

    for( int x = 0; x < cols; ++x ) {
      const int fx = (x > cols / 2) ? (cols - x) : x;

      // ИСПРАВЛЕНО: Вычисляем логический знак fftShift.
      // Для оси x=0 и Найквиста по X, чётные и нечётные строки в CCS склеены в одну частоту.
      // Чтобы фильтр был универсальным, знак знаковой маски на вертикальных осях
      // должен определяться логической частотой y_logical, а не физической строкой y!
      int sign = 1;
      if (x == 0 || (cols % 2 == 0 && x == cols / 2)) {
        // На вертикальных осях логический индекс частоты определяется по правилу CCS:
        if (y == 0 || (rows % 2 == 0 && y == rows / 2)) {
          sign = ((x + y) & 1) ? -1 : 1;
        } else {
          // Для комплексных пар частота равна (y + 1) / 2 для нечетных и y / 2 для четных.
          // Но так как они делят ОДНУ частоту, мы берем индекс по строке вещественной части (нечетной):
          int y_logical = (y % 2 != 0) ? y : (y - 1);
          sign = ((x + y_logical) & 1) ? -1 : 1;
        }
      } else {
        // Для всего остального поля спектра (x > 0) обычное шахматное чередование
        sign = ((x + y) & 1) ? -1 : 1;
      }

      const float u = fx * inv_cols;
      const float u2 = u * u;
      const float rho2 = (u2 + v2) * lambda2;

      fltp[x] = sign * rho2 * std::exp(-0.5f * rho2);
    }
  }
});
#endif

//void c_phase_correlate::generateBandpassFilter()
//{
//  // fsigma = sqrt(2)/ (CV_PI * _gsigma)
//  // rho2 = (u^2 + v^2) / fsigma^2;
//  // F(u, v) = rho2 * exp (-0.5 * rho2 )
//
//  _bandpassFilter.create(_fftSize);
//
//  const uint8_t * filter_base = _bandpassFilter.ptr();
//  const size_t filter_stride = _bandpassFilter.step;
//
//  const int cols = _fftSize.width;
//  const int rows = _fftSize.height;
//
//  if ( _gsigma <= 0 ) {
//    parallel_for(0, rows, [=](const auto & range) {
//      for( int y = rbegin(range); y < rend(range); ++y ) {
//        float * __restrict fltp = (float * )(filter_base + y * filter_stride);
//        const float start_sign = (y & 1) ? -1.0f : 1.0f;
//        for( int x = 0; x < cols; ++x ) {
//          fltp[x] = (x & 1) ? -start_sign : start_sign;
//        }
//      }
//    });
//  }
//  else {
//    const float inv_cols = float (1.0 / cols);
//    const float inv_rows = float (1.0 / rows);
//    const float lambda2 = float(0.5 * CV_PI * CV_PI * _gsigma * _gsigma);
//
//    parallel_for(0, rows, [=](const auto & range) {
//      for( int y = rbegin(range); y < rend(range); ++y ) {
//        float * __restrict fltp = (float * )(filter_base + y * filter_stride);
//
//        const int fy = (y > rows / 2) ? (rows - y) : y;
//        const float v = fy * inv_rows;
//        const float v2 = v * v;
//
//        for( int x = 0; x < cols; ++x ) {
//          const int fx = (x > cols / 2) ? (cols - x) : x;
//
//          // ИСПРАВЛЕНО: Вычисляем логический знак fftShift.
//          // Для оси x=0 и Найквиста по X, чётные и нечётные строки в CCS склеены в одну частоту.
//          // Чтобы фильтр был универсальным, знак знаковой маски на вертикальных осях
//          // должен определяться логической частотой y_logical, а не физической строкой y!
//          int sign = 1;
////          if (x == 0 || (cols % 2 == 0 && x == cols / 2)) {
////            // На вертикальных осях логический индекс частоты определяется по правилу CCS:
////            if (y == 0 || (rows % 2 == 0 && y == rows / 2)) {
////              sign = ((x + y) & 1) ? -1 : 1;
////            } else {
////              // Для комплексных пар частота равна (y + 1) / 2 для нечетных и y / 2 для четных.
////              // Но так как они делят ОДНУ частоту, мы берем индекс по строке вещественной части (нечетной):
////              int y_logical = (y % 2 != 0) ? y : (y - 1);
////              sign = ((x + y_logical) & 1) ? -1 : 1;
////            }
////          } else {
////            // Для всего остального поля спектра (x > 0) обычное шахматное чередование
////            sign = ((x + y) & 1) ? -1 : 1;
////          }
//
//          const float u = fx * inv_cols;
//          const float u2 = u * u;
//          const float rho2 = (u2 + v2) * lambda2;
//
//          fltp[x] = sign * rho2 * std::exp(-0.5f * rho2);
//        }
//      }
//    });
//  }
//
//  if ( _csigma > 0  && _calpha > 0) {
//    fftGenerateInverseCrossFilter(_fftSize, _expectedFrameSize, _crossMask, _csigma, _calpha, false);
//    cv::multiply(_bandpassFilter, _crossMask, _bandpassFilter);
//  }
//
//  cv::multiply(_bandpassFilter, 1. / cv::norm(_bandpassFilter, cv::NORM_L1),
//      _bandpassFilter);
//}

void c_phase_correlate::generateBandpassFilter()
{
  // fsigma = sqrt(2)/ (CV_PI * _gsigma)
  // rho2 = (u^2 + v^2) / fsigma^2;
  // F(u, v) = rho2 * exp (-0.5 * rho2 )

  _bandpassFilter.create(_fftSize);

  const uint8_t * filter_base = _bandpassFilter.ptr();
  const size_t filter_stride = _bandpassFilter.step;

  const int cols = _fftSize.width;
  const int rows = _fftSize.height;

  if ( _gsigma <= 0 ) {
    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);
        const float start_sign = (y & 1) ? -1.0f : 1.0f;
        for( int x = 0; x < cols; ++x ) {
          fltp[x] = (x & 1) ? -start_sign : start_sign;
        }
      }
    });
  }
  else {

    const float inv_cols = float (1.0 / cols);
    const float inv_rows = float (1.0 / rows);
    const float lambda2 = float(0.5 * CV_PI * CV_PI * _gsigma * _gsigma);

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);

        const int fy = (y > rows / 2) ? (rows - y) : y;
        const float v = fy * inv_rows;
        const float v2 = v * v;

        for( int x = 0; x < cols; ++x ) {
          const int fx = (x > cols / 2) ? (cols - x) : x;
          const int sign = ((x + y) & 1) ? -1 : 1;

          const float u = fx * inv_cols;
          const float u2 = u * u;
          const float rho2 = (u2 + v2) * lambda2;

          fltp[x] = sign * rho2 * std::exp(-0.5f * rho2);
        }
      }
    });
  }

  if ( _csigma > 0  && _calpha > 0) {
    fftGenerateInverseCrossFilter(_fftSize, _expectedFrameSize, _crossMask, _csigma, _calpha, false);
    cv::multiply(_bandpassFilter, _crossMask, _bandpassFilter);
  }

  cv::multiply(_bandpassFilter, 1. / cv::norm(_bandpassFilter, cv::NORM_L1),
      _bandpassFilter);
}

bool c_phase_correlate::setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  if( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( referenceImage.empty() ) {
    CF_ERROR("c_phase_correlate: referenceImage is empty");
    return false;
  }

  if ( referenceImage.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: referenceImage must be single-channel");
    return false;
  }

  if (!referenceMask.empty() && referenceMask.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: referenceMask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;

  const double scale = 1.0 / _downscale_factor;
  cv::resize(referenceImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  if (!referenceMask.empty()) {
    cv::resize(referenceMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  }

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      _scaledReferenceImage, _scaledReferenceMask, _fftSize,
      _referenceValidSize,
      _referenceCropOffset);

  cv::dft(_scaledReferenceImage, _referenceSpectrum,
      cv::DFT_REAL_OUTPUT);

  return true;
}

bool c_phase_correlate::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  if( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( currentImage.empty() ) {
    CF_ERROR("c_phase_correlate: currentImage is empty");
    return false;
  }

  if ( currentImage.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: currentImage must be single-channel");
    return false;
  }

  if (!currentMask.empty() && currentMask.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: currentMask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;
  const double scale = 1.0 / _downscale_factor;
  cv::resize(currentImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  if (!currentMask.empty()) {
    cv::resize(currentMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  }

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      _scaledCurrentImage, _scaledCurrentMask, _fftSize,
      _currentValidSize,
      _currentCropOffset);

  cv::dft(_scaledCurrentImage, _currentSpectrum,
      cv::DFT_REAL_OUTPUT);

  return true;
}

bool c_phase_correlate::computeCorrelationMap()
{
#if 1
  const bool fOK =
      fftCrossSpectrumPhaseCorrelateWeightedCCS(_currentSpectrum, _referenceSpectrum,
          _bandpassFilter, _crossSpectrum);
  if( !fOK ) {
    CF_ERROR("fftCrossSpectrumPhaseCorrelateWeightedCCS() fails");
    return false;
  }
#else
  cv::mulSpectrums(_currentSpectrum, _referenceSpectrum, _crossSpectrum, 0, true);
#endif

  cv::idft(_crossSpectrum, _correlationMap,
      cv::DFT_REAL_OUTPUT);

  return true;
}

double c_phase_correlate::compute(cv::Vec2f & outputTranslation)
{
  if (_fftSize.empty() || _currentSpectrum.size() != _fftSize || _referenceSpectrum.size() != _fftSize ) {
    CF_ERROR("c_phase_correlate: was not properly initialized with setup()");
    return -1;
  }

  if ( !computeCorrelationMap() ) {
    CF_ERROR("computeCorrelationMap() fails");
    return -1;
  }

  cv::Point2f peakPos;

  _peakValue =
      findSubpixelCentroid(_correlationMap,
          peakPos);

  // Compensate for image resolution scale
  const double scaledDx = peakPos.x - _fftSize.width / 2 + _referenceCropOffset.x - _currentCropOffset.x;
  const double scaledDy = peakPos.y - _fftSize.height / 2 + _referenceCropOffset.y - _currentCropOffset.y;
  outputTranslation[0] = float(-scaledDx * _downscale_factor);
  outputTranslation[1] = float(-scaledDy * _downscale_factor);

  // Compensate correlation score for shifted frame overlap
  const double kx = 1.0 - std::abs(outputTranslation[0]) / (_fftSize.width * _downscale_factor);
  const double ky = 1.0 - std::abs(outputTranslation[1]) / (_fftSize.height * _downscale_factor);
  const double area_rel = kx * ky;
  const double k_min_axis = std::min(kx, ky);
  const double dynamic_eps = std::exp(-70.0 * (k_min_axis - 0.25));

  return (_correlationScore = _peakValue / (area_rel + dynamic_eps));
}

double c_phase_correlate::findSubpixelCentroid(const cv::Mat1f& correlationMap, cv::Point2f & peakPos) const
{
  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  int maxIdx[2] = {0, 0};
  cv::minMaxIdx(correlationMap, nullptr, nullptr, nullptr, maxIdx);

  const int y0 = maxIdx[0];
  const int x0 = maxIdx[1];
  if (x0 <= 0 || x0 >= cols - 1 || y0 <= 0 || y0 >= rows - 1) {
    peakPos = cv::Point2f(float(x0), float(y0));
    return -1;
  }

  // Least-squares approximation of a paraboloid over a 3x3 neighborhood
  const float z_00 = correlationMap(y0 - 1, x0 - 1); // Top-left
  const float z_10 = correlationMap(y0 - 1, x0); // Top-center
  const float z_20 = correlationMap(y0 - 1, x0 + 1); // Top-right

  const float z_01 = correlationMap(y0,     x0 - 1); // Middle-left
  const float z_11 = correlationMap(y0,     x0); // True center (peak)
  const float z_21 = correlationMap(y0,     x0 + 1); // Middle-right

  const float z_02 = correlationMap(y0 + 1, x0 - 1); // Bottom-left
  const float z_12 = correlationMap(y0 + 1, x0); // Bottom-center
  const float z_22 = correlationMap(y0 + 1, x0 + 1); // Bottom-right

  const float C = (z_20 + z_21 + z_22 - (z_00 + z_01 + z_02)) / 6.0f;
  const float D = (z_02 + z_12 + z_22 - (z_00 + z_10 + z_20)) / 6.0f;
  const float A = (z_00 + z_01 + z_02 + z_20 + z_21 + z_22) / 6.0f - (z_10 + z_11 + z_12) / 3.0f;
  const float B = (z_00 + z_10 + z_20 + z_02 + z_12 + z_22) / 6.0f - (z_01 + z_11 + z_21) / 3.0f;


  // Parabola extremum: delta = -derivative / (2 * curvature)
  // deltaX = -C / (2*A), deltaY = -D / (2*B)
  const float adaptive_thresh = std::abs(z_11) * 1e-6f;
  const float deltaX = std::abs(A) > adaptive_thresh ? -C / (2.0f * A) : 0.0f;
  const float deltaY = std::abs(B) > adaptive_thresh ? -D / (2.0f * B) : 0.0f;

  peakPos.x = float(x0 + deltaX);
  peakPos.y = float(y0 + deltaY);

  return correlationMap(y0, x0);
}


bool serialize_phase_correlate_options(c_config_setting section, bool save,
    c_phase_correlate_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, downscale_factor);
  SERIALIZE_OPTION(section, save, opts, gsigma);
  SERIALIZE_OPTION(section, save, opts, csigma);
  SERIALIZE_OPTION(section, save, opts, calpha);
  return true;
}


