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

/**
 * @brief Prepares and aligns a downscaled image and its optional mask into fixed-size FFT buffers.
 * @details Extracts a centered Region of Interest (ROI) from the source image if its dimensions
 *          exceed the target FFT grid size. It then handles zero-padding (if the input is smaller
 *          than @p fftSize), type conversion to floating-point representation (`CV_32F`), and
 *          thresholding of the mask matrix down to standard binary format (`CV_8U`).
 *
 * @param[in]  srcImage        Source downscaled image matrix.
 * @param[in]  srcMask         Source optional alignment mask matrix.
 * @param[out] outImage        Output zero-padded floating-point matrix resized to fit @p fftSize.
 * @param[out] outMask         Output binary mask matrix resized to fit @p fftSize (filled with 255 if @p srcMask is empty).
 * @param[in]  fftSize         Target fixed layout dimensions optimal for Fast Fourier Transform operations.
 * @param[out] outputValidSize The actual unpadded dimensions of the processed frame data within the padding window.
 * @param[out] cropOffset      Pixel displacement offset from the original frame center if cropping was applied.
 */
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

static int getOptimalFFTSizeDown(int size)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  int sopt = cv::getOptimalDFTSize(size);
  while ( sopt > 0 && ((sopt > size) || (sopt & 0x1)) ) {
    sopt = cv::getOptimalDFTSize(--size);
  }
  return sopt;
}

cv::Size c_phase_correlate::computeFFTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  const int downscaledW = getOptimalFFTSizeDown(cvRound(expectedFrameSize.width / downscaleFactor));
  const int downscaledH = getOptimalFFTSizeDown(cvRound(expectedFrameSize.height / downscaleFactor));
  return cv::Size(std::max(4, downscaledW), std::max(4, downscaledH));
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

  // pre-generate bandpass filter for expected frame size
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
  _initialized = false;
}

/**
 * @brief Pre-calculates the frequency bandpass weighting filter matrix based on pipeline options.
 * @details Generates a 2D bandpass filter grid directly mapped to the @ref _fftSize.
 *          The method utilizes multi-threaded execution via OpenMP/TBB (`parallel_for`) and embeds
 *          spatial chessboard sign-alternation (`((x + y) & 1) ? -1 : 1`) to inherently shift
 *          the spectrum origin to the center, bypassing the performance overhead of an explicit
 *          `fftShift` loop.
 *
 *          If `@ref _gsigma <= 0`, a fallback high-pass/alternating matrix is generated. Otherwise,
 *          a Rayleigh-like frequency distribution curve is evaluated. If `@ref _csigma > 0` and
 *          `@ref _calpha > 0`, an additional inverse cross-filter mask for mask-edge or blur
 *          deconvolution is combined with the map. The final matrix is fully normalized via the L1 norm.
 *
 */
void c_phase_correlate::generateBandpassFilter()
{
  // fsigma = sqrt(2)/ (CV_PI * _gsigma)
  // rho2 = (u^2 + v^2) / fsigma^2;
  // F(u, v) = rho2 * exp (-0.5 * rho2 )
  // the alternating +1 and -1 is also inserted to avoid later fftSwapQudrants()

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
      cv::DFT_REAL_OUTPUT|cv::DFT_SCALE);

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
  const bool fOK =
      fftCrossSpectrumPhaseCorrelateWeightedCCS(_currentSpectrum, _referenceSpectrum,
          _bandpassFilter, _crossSpectrum);

  if( !fOK ) {
    CF_ERROR("fftCrossSpectrumPhaseCorrelateWeightedCCS() fails");
    return false;
  }

  cv::idft(_crossSpectrum, _correlationMap,
      cv::DFT_REAL_OUTPUT); // |cv::DFT_SCALE

  return true;
}

/**
 * @brief Computes phase correlation and estimates the precise 2D translation vector.
 * @details Executes cross-spectrum phase evaluation, performs an inverse DFT, interpolates
 *          the subpixel peak, and shifts the result back to original unscaled pixel units,
 *          accounting for downscaling and crop offsets.
 *
 * @param[out] outputTranslation Resulting [dx, dy] translation vector in original pixels.
 * @return Overlap-compensated correlation quality score (PSR-like metric), or -1 on failure.
 */
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
  const double dynamic_eps = std::exp(-70.0 * (k_min_axis - 0.3));

  return (_correlationScore = _peakValue / (area_rel + dynamic_eps));
}

/**
 * @brief Locate the subpixel translation peak using a non-linear 5x5 centroid window.
 * @details Finds the global integer maximum and refines its location by computing a
 *          weighted center of mass over a 5x5 neighborhood. Applies an adaptive threshold
 *          to isolate the peak from bandpass sidelobes, and utilizes a cubic weighting
 *          scheme to eliminate pixel-locking effects and ensure robustness against
 *          atmospheric turbulence or temporary image splitting.
 *
 * @param correlationMap Input 2D real-valued cross-correlation map after inverse DFT.
 * @param peakPos Output refined subpixel coordinates of the correlation peak.
 * @return The absolute peak value at the integer maximum location.
 */
double c_phase_correlate::findSubpixelCentroid(const cv::Mat1f& correlationMap, cv::Point2f & peakPos) const
{
  // Adaptive threshold 20% of the peak cuts off well the filter's sidelobes.
  // Cubic weight (val - threshold)^3
  // Works well on flat peaks in a turbulent environment

  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  int maxIdx[2] = {0, 0};
  cv::minMaxIdx(correlationMap, nullptr, nullptr, nullptr, maxIdx);

  constexpr int R = 2;
  const int y0 = maxIdx[0];
  const int x0 = maxIdx[1];

  if (x0 < R || x0 >= cols - R || y0 < R || y0 >= rows - R) {
    peakPos = cv::Point2f(float(x0), float(y0));
    return -1;
  }

  const float z_center = correlationMap(y0, x0);
  const double threshold = z_center * 0.20f;
  double sumWeights = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;

  const uint8_t * src_base = correlationMap.ptr(y0);
  const size_t stride = correlationMap.step;
  for (int dy = -R; dy <= R; ++dy) {
    const float * __restrict srcp = (const float *)(src_base + dy * stride);
    for (int dx = -R; dx <= R; ++dx) {
      const double val = srcp[x0 + dx];
      if (val > threshold) {
        const double diff = val - threshold;
        const double w = diff * diff * diff;
        sumWeights += w;
        sumX += dx * w;
        sumY += dy * w;
      }
    }
  }

  if( sumWeights > 1e-9 ) {
    peakPos.x = float(x0 + sumX / sumWeights);
    peakPos.y = float(y0 + sumY / sumWeights);
  }
  else {
    peakPos = cv::Point2f(float(x0), float(y0));
  }

  return z_center;
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


