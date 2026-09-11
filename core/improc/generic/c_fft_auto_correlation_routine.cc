/*
 * c_fft_auto_correlation_routine.cc
 *
 *  Created on: Sep 11, 2026
 *      Author: amyznikov
 */

#include "c_fft_auto_correlation_routine.h"
#include <core/proc/run-loop.h>
#include <core/proc/fft.h>

template<>
const c_enum_member * members_of<c_fft_auto_correlation_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_auto_correlation_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_fft_auto_correlation_routine::DISPLAY_CURRENT_SCALED_IMAGE, "SCALED_IMAGE"},
      { c_fft_auto_correlation_routine::DISPLAY_CURRENT_SPECTRUM_CART,"IMAGE_SPECTRUM_CART"},
      { c_fft_auto_correlation_routine::DISPLAY_CURRENT_SPECTRUM_POLAR,"IMAGE_SPECTRUM_POLAR"},
      { c_fft_auto_correlation_routine::DISPLAY_CROSS_SPECTRUM_CART, "CROSS_SPECTRUM_CART", "" },
      { c_fft_auto_correlation_routine::DISPLAY_CROSS_SPECTRUM_POLAR, "CROSS_SPECTRUM_POLAR", "" },
      { c_fft_auto_correlation_routine::DISPLAY_CORRELATION_MAP,"CORRELATION_MAP"},
      { c_fft_auto_correlation_routine::DISPLAY_CORRELATION_MAP}
  };
  return members;
}

/////////////////////////////////////
namespace {

static cv::Size computeFFTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor)
{
  // Find the closest power of two (round mathematically to the nearest)
  // cvRound(std::log2(v)) will select the power that is closest to the target
  // Some limit from below (for example not less than 64 pixels, so that the algorithm does not degenerate)
  // Return the size as 2^powX and 2^powY

  const int downscaledW = cvRound(expectedFrameSize.width / downscaleFactor);
  const int downscaledH = cvRound(expectedFrameSize.height / downscaleFactor);
  const int powX = std::max(6, cvCeil(std::log2(downscaledW)));
  const int powY = std::max(6, cvCeil(std::log2(downscaledH)));
  return cv::Size(1 << powX, 1 << powY);
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


} // namespace

bool c_fft_auto_correlation_routine::ensureInitialized(const cv::Size & expectedFrameSize)
{
  if( expectedFrameSize.empty() ) {
    CF_ERROR("BAD expectedFrameSize specified : %dx%d",
        expectedFrameSize.width, expectedFrameSize.height);
    return false;
  }

  if( _downscaleFactor < 1 ) {
    CF_ERROR("downscale_factor specified : %g. Must be >= 1", _downscaleFactor);
    return false;
  }

  const cv::Size fftSize = computeFFTPackSize(expectedFrameSize, _downscaleFactor);
  if( !_initialized || fftSize != _fftSize ) {

    _fftSize = fftSize;

    if( _apodizationSize < 1 ) {
      _apodizationLUT.clear();
    }
    else {
      // Smoothly increases from 0.0 to 1.0
      const int ksize = _apodizationSize;
      _apodizationLUT.resize(ksize + 1);
      for( int i = 0; i <= ksize; ++i ) {
        const float t = float(i) / ksize;
        _apodizationLUT[i] = t * t * (3.0f - 2.0f * t);
      }
    }

    _initialized = true;
  }

  return _initialized;
}

bool c_fft_auto_correlation_routine::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  if( _fftSize.empty() ) {
    CF_ERROR("c_fft_auto_correlation: was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( currentImage.empty() ) {
    CF_ERROR("c_fft_auto_correlation: currentImage is empty");
    return false;
  }

  if ( currentImage.channels() == 1 ) {
    currentImage.getMat().convertTo(_currentImage, CV_32F);
  }
  else {
    cv::Mat tmp;
    cv::cvtColor(currentImage, tmp, cv::COLOR_BGR2GRAY);
    tmp.convertTo(_currentImage, CV_32F);
  }

  if (!currentMask.empty() && currentMask.channels() != 1 ) {
    CF_ERROR("c_fft_auto_correlation: currentMask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;
  const double scale = 1.0 / _downscaleFactor;
  cv::resize(currentImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  if (!currentMask.empty()) {
    cv::resize(currentMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  }

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      _scaledCurrentImage, _scaledCurrentMask, _fftSize,
      _currentValidSize,
      _currentCropOffset);

  if (_apodizationSize > 0) {
    applyApodization(_scaledCurrentImage, _scaledCurrentMask,
        _currentValidSize);
  }

  cv::dft(_scaledCurrentImage, _currentSpectrum,
      cv::DFT_REAL_OUTPUT);

  return true;
}

void c_fft_auto_correlation_routine::applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask,
    const cv::Size & validSize)
{
  if (_apodizationLUT.empty()) {
    return;
  }

  const auto & lut = _apodizationLUT;
  const int ksize = lut.size() - 1;

  cv::Mat1b activeMask = scaledMask(cv::Rect(0, 0, validSize.width, validSize.height));

  const int totalPixels = validSize.width * validSize.height;
  const bool isFullMask = (cv::countNonZero(activeMask) == totalPixels);

  if( isFullMask ) {
    parallel_for(0, validSize.height, [&](const auto & range) {
       for (int y = rbegin(range); y < rend(range); ++y) {
         float * imgp = scaledImage.ptr<float>(y);
         const float dist_y = std::min(y, validSize.height - 1 - y);
         for (int x = 0; x < validSize.width; ++x) {
           const float dist_x = std::min(x, validSize.width - 1 - x);
           const float d = std::min(dist_x, dist_y);
           if (d < ksize) {
             const int idx = int(d);
             const float fract = d - idx;
             imgp[x] *= (lut[idx] + fract * (lut[idx + 1] - lut[idx]));
           }
         }
       }
     });
  }
  else {
    cv::distanceTransform(scaledMask, _distmap, cv::DIST_L2, cv::DIST_MASK_3);

    parallel_for(0, validSize.height, [&](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        const float* distp = _distmap[y];
        float * __restrict imgp = scaledImage[y];
        for (int x = 0; x < validSize.width; ++x) {
          const float d = distp[x];
          if (d <= 0) {
            imgp[x] = 0.0f;
          }
          else if (d < ksize) {
            const int idx = int(d);
            const float fract = d - idx;
            const float v = imgp[x];
            imgp[x] = v * (lut[idx] + fract * (lut[idx + 1] - lut[idx]));
          }
        }
      }
    });
  }
}


bool c_fft_auto_correlation_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _downscaleFactor);
    SERIALIZE_OPTION(settings, save, *this, _gsigma);
    SERIALIZE_OPTION(settings, save, *this, _apodizationSize);
    return true;
  }
  return false;
}

void c_fft_auto_correlation_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "downscaleFactor", ctx,  &this_class::downscaleFactor, &this_class::set_downscaleFactor, "");
  ctlbind(ctls, "gsigma", ctx,  &this_class::gsigma, &this_class::set_gsigma, "");
  ctlbind(ctls, "apodizationSize", ctx,  &this_class::apodizationSize, &this_class::set_apodizationSize, "");
}

double c_fft_auto_correlation_routine::computeCorrelationMap()
{
  // The current spectrum is expected in
  // CCS (Complex-Conjugate Symmetrical) format computed
  // from real images with cv::DFT_REAL_OUTPUT flag
  //
  // The cross spectrum is weighted by bandpass filter:
  // w(f) ~ f^2 * exp(-0.5 * f^2 / sigma_f^2)
  //
  // The _gsigma is passed as the characteristic scale lambda in pixels, e.g., 10.0f
  // The normalization factor makes the filter peak w(f_max) equal to 1.0:
  //  w(f_max) = 1 / (e * lambda_max^2) -> norm_factor = e * lambda_max^2
  //  e = 2.71828183f
  //
  // Additionally the cross spectrum it is multiplied by alternating +1 and -1
  // to avoid fftSwapQuadrants() after idft()

  const int rows = _currentSpectrum.rows;
  const int cols = _currentSpectrum.cols;
  const bool is_even = (cols % 2 == 0);
  const float inv_cols = float(1.0 / cols);
  const float inv_rows = float(1.0f / rows);
  const float lambda_max = float(_gsigma);

  const bool unique_weight = (lambda_max > 0.1f);
  const float inv_gsigma2 = unique_weight ? -(lambda_max * lambda_max) : 0.0f;

  std::vector<float> lut_exp_u(cols, 1.0f);
  if (unique_weight) {
    const int max_complex_idx = is_even ? (cols - 2) : (cols - 1);
    for (int x = 1; x <= max_complex_idx; x += 2) {
      const int fx = (x + 1) / 2;
      const float u = fx * inv_cols;
      const float exp_u = std::exp(u * u * inv_gsigma2);
      lut_exp_u[x] = exp_u;
      lut_exp_u[x + 1] = exp_u;
    }
  }

  _crossSpectrum.create(_currentSpectrum.size());

  const uint8_t * spec_base = _currentSpectrum.ptr();
  const size_t spec_stride = _currentSpectrum.step;

  uint8_t * cross_base = _crossSpectrum.ptr();
  const size_t cross_stride = _crossSpectrum.step;

  const float * exp_u = lut_exp_u.data();

  alignas(std::hardware_destructive_interference_size)
    std::atomic<float> total_energy(0.0f);

  parallel_for(0, rows, [=, &total_energy](const auto & range) {
    float local_energy = 0.0f;

    for( int y = rbegin(range); y < rend(range); ++y ) {
      const float * srcp1 = (const float * )(spec_base + y * spec_stride);
      float * __restrict dstp = (float *)(cross_base + y * cross_stride);

      const float v = ((y > rows / 2) ? (rows - y) : y ) * inv_rows;
      const float exp_v = unique_weight ? std::exp(v * v * inv_gsigma2) : 1;
      const float v2 = v * v;
      const float sign_y = (y % 2 == 0) ? 1.0f : -1.0f;

      dstp[0] = 0.0f;

      const int max_complex_idx = is_even ? (cols - 2) : (cols - 1);

      for( int x = 1; x <= max_complex_idx; x += 2 ) {
        const int fx = (x + 1) / 2;
        const float sign = ((fx + y) % 2 == 0) ? 1.0f : -1.0f;
        const float u = fx * inv_cols;
        const float u2 = u * u;
        const float rho2 = u2 + v2;
        const float gw = unique_weight ? (rho2 * exp_v * exp_u[x]) : 1;
        const float a = srcp1[x];
        const float b = srcp1[x + 1];
        const float mag2 = gw * (a * a + b * b);
        local_energy += mag2 * mag2;
        dstp[x + 0] = mag2 * sign;
        dstp[x + 1] = 0.0f;
      }
      if( is_even ) {
        dstp[cols - 1] = 0.0f;
      }
    }

    float current = total_energy.load(std::memory_order_relaxed);
    while (!total_energy.compare_exchange_weak(current, current + local_energy,
          std::memory_order_relaxed));
  });

  const double total_bandpass_energy =
      total_energy.load();

  cv::idft(_crossSpectrum, _correlationMap,
      cv::DFT_REAL_OUTPUT);

  return total_bandpass_energy;
}

bool c_fft_auto_correlation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !ensureInitialized(image.size()) )  {
    CF_ERROR("reinitialize() fails");
    return false;
  }

  if ( !setCurrentImage(image, mask) ) {
    CF_ERROR("setCurrentImage() fails");
    return false;
  }

  const double spectrum_energy =
      computeCorrelationMap();

  switch (_display) {
    case DISPLAY_CORRELATION_MAP: {
      _correlationMap.convertTo(image, CV_32F, 1. / std::sqrt(spectrum_energy));
      mask.release();
      break;
    }
    case DISPLAY_CURRENT_IMAGE: {
      _currentImage.copyTo(image);
      mask.release();
      break;
    }
    case DISPLAY_CURRENT_SCALED_IMAGE: {
      _scaledCurrentImage.copyTo(image);
      _scaledCurrentMask.copyTo(mask);
      break;
    }
    case DISPLAY_CURRENT_SPECTRUM_CART: {
      fftUnpackCCSSpectrum(_currentSpectrum, image);
      mask.release();
      break;
    }
    case DISPLAY_CURRENT_SPECTRUM_POLAR:  {
      fftUnpackCCSSpectrum(_currentSpectrum, image);
      fftSpectrumToPolar(image, image);
      mask.release();
      break;
    }
    case DISPLAY_CROSS_SPECTRUM_CART: {
      fftUnpackCCSSpectrumAlternateSign(_crossSpectrum, image);
      mask.release();
      break;
    }
    case DISPLAY_CROSS_SPECTRUM_POLAR: {
      fftUnpackCCSSpectrumAlternateSign(_crossSpectrum, image);
      fftSpectrumToPolar(image, image);
      mask.release();
      break;
    }
    default:
      break;
  }

  return true;
}

