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
      { c_fft_auto_correlation_routine::DISPLAY_FILTER, "FILTER"},
      { c_fft_auto_correlation_routine::DISPLAY_INVERSE_CROSS, "INVERSE_CROSS"},
      { c_fft_auto_correlation_routine::DISPLAY_CORRELATION_MAP}
  };
  return members;
}

/////////////////////////////////////
namespace {

static int getOptimalFFTSizeDown(int size)
{
  int sopt = cv::getOptimalDFTSize(size);
  while ( sopt > 0 && sopt > size ) {
    sopt = cv::getOptimalDFTSize(--size);
  }
  return sopt;
}


static cv::Size computeFFTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor)
{
  const int downscaledW = getOptimalFFTSizeDown(cvRound(expectedFrameSize.width / downscaleFactor));
  const int downscaledH = getOptimalFFTSizeDown(cvRound(expectedFrameSize.height / downscaleFactor));
  return cv::Size(std::max(4,downscaledW), std::max(4,downscaledH));
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

    if ( !_currentImage.empty() ) {
      generateBandpassFilter();
    }

    _initialized = true;
  }

  return _initialized;
}

void c_fft_auto_correlation_routine::generateBandpassFilter()
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
    const double sgsigma = _gsigma / _downscaleFactor;
    const float lambda2 = float(0.5 * CV_PI * CV_PI * sgsigma * sgsigma);

    const float inv_cols = float (1.0 / cols);
    const float inv_rows = float (1.0 / rows);

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

  if( _csigma > 0 && _calpha > 0 ) {
    fftGenerateInverseCrossFilter(_fftSize, _currentValidSize, _inverseCross, _csigma, _calpha, false);
    if ( !_inverseCross.empty()  ) {
      cv::multiply(_bandpassFilter, _inverseCross, _bandpassFilter);
    }
  }

  _bandpassFilterNorm = cv::norm(_bandpassFilter, cv::NORM_L1);
  cv::multiply(_bandpassFilter, 1. / _bandpassFilterNorm, _bandpassFilter);
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

void c_fft_auto_correlation_routine::computeCorrelationMap()
{
  if (_bandpassFilter.empty() ) {
    generateBandpassFilter();
  }

  _crossSpectrumEnergy =
      fftAutoCrossSpectrumWeightedCCS(_currentSpectrum,
          _bandpassFilter,
          _autoCrossSpectrum);

  cv::idft(_autoCrossSpectrum, _autoCorrelationMap,
      cv::DFT_REAL_OUTPUT);
}

bool c_fft_auto_correlation_routine::analyzeSpotGeometry()
{
  const int rows = _autoCorrelationMap.rows;
  const int cols = _autoCorrelationMap.cols;

  // Autocorrelation peak strictly at the center
  const int cx = cols / 2;
  const int cy = rows / 2;

  // Approximate spot radius for given bbandpass filter sigma.
  const double sgsigma = _gsigma / _downscaleFactor;
  const double Rspot = 0.5 * sgsigma;

  // ROI around the central pixel
  const int R = std::max(3, cvCeil(Rspot));
  const int RSIZE = 2 * R + 1;
  const cv::Rect ROI = cv::Rect(cx - R, cy - R, RSIZE, RSIZE) & cv::Rect(0, 0, cols, rows);
  const cv::Mat1f roi = _autoCorrelationMap(ROI);

  cv::Mat1f croi;
  cv::subtract(roi, cv::mean(roi), croi);
  cv::max(croi, 0, croi);
  const cv::Moments m = cv::moments(croi, false);

  static constexpr float safety_thresh =
      std::numeric_limits<float>::min();

  _spot.radius = R;
  _spot.peak = 1024.0 * _autoCorrelationMap(cy, cx) / std::sqrt(_fftSize.area() * _crossSpectrumEnergy);

  if( m.m00 > safety_thresh ) {
    const double mu20 = m.mu20 / m.m00;
    const double mu02 = m.mu02 / m.m00;
    const double mu11 = m.mu11 / m.m00;

    // Eigen values
    const double delta = mu20 - mu02;
    const double sqrt_term = std::sqrt(delta * delta + 4.0 * mu11 * mu11);
    const double lambda1 = 0.5 * (mu20 + mu02 + sqrt_term);
    const double lambda2 = 0.5 * (mu20 + mu02 - sqrt_term);

    // FWHM
    const double sigma_to_fwhm = 2.35482004503;
    _spot.fwhm_x = std::sqrt(std::max(0.0, lambda1)) * sigma_to_fwhm; // Major
    _spot.fwhm_y = std::sqrt(std::max(0.0, lambda2)) * sigma_to_fwhm; // Minor

    // Angle
    if( std::abs(delta) > safety_thresh || std::abs(mu11) > safety_thresh ) {
      _spot.angle = 0.5 * std::atan2(2.0 * mu11, delta) * (180.0 / CV_PI);
    }
    else {
      _spot.angle = 0.0;
    }

    // eccentricity: 0.0 – perfect circle, closer to 1.0 – elongated oval
    if( lambda1 > safety_thresh ) {
      _spot.eccentricity = std::sqrt(std::max(0.0, 1.0 - (lambda2 / lambda1)));
    }
    else {
      _spot.eccentricity = 0.0;
    }

    return true;
  }

  return false;
}

bool c_fft_auto_correlation_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _downscaleFactor);
    SERIALIZE_OPTION(settings, save, *this, _gsigma);
    SERIALIZE_OPTION(settings, save, *this, _csigma);
    SERIALIZE_OPTION(settings, save, *this, _calpha);
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
  ctlbind(ctls, "csigma", ctx,  &this_class::csigma, &this_class::set_csigma, "");
  ctlbind(ctls, "calpha", ctx,  &this_class::calpha, &this_class::set_calpha, "");
  ctlbind(ctls, "apodizationSize", ctx,  &this_class::apodizationSize, &this_class::set_apodizationSize, "");
  ctlbind(ctls, "print debug", CTL_CONTEXT(ctx, _printDebug), "");
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

  computeCorrelationMap();

  if( _printDebug ) {
    if( !analyzeSpotGeometry() ) {
      CF_ERROR("analyzeSpotGeometry() fails");
    }
    else {
      CF_DEBUG("\nSpot: R=%3g peak = %8.3f fwhm = %7.3f x%7.3f angle = %7.3f ecc = %6.3f\n",
          _spot.radius, _spot.peak, _spot.fwhm_x, _spot.fwhm_y, _spot.angle, _spot.eccentricity);
    }
  }

  switch (_display) {
    case DISPLAY_CORRELATION_MAP: {
      const double combinedScale = 1024.0 / std::sqrt(_fftSize.area() * _crossSpectrumEnergy);
      _autoCorrelationMap.convertTo(image, CV_32F, combinedScale);
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
      fftSwapQuadrants(image, image);
      mask.release();
      break;
    }
    case DISPLAY_CURRENT_SPECTRUM_POLAR:  {
      fftUnpackCCSSpectrum(_currentSpectrum, image);
      fftSpectrumToPolar(image, image);
      fftSwapQuadrants(image, image);
      mask.release();
      break;
    }
    case DISPLAY_CROSS_SPECTRUM_CART: {
      fftUnpackCCSSpectrumAlternateSign(_autoCrossSpectrum, image);
      fftSwapQuadrants(image, image);
      mask.release();
      break;
    }
    case DISPLAY_CROSS_SPECTRUM_POLAR: {
      fftUnpackCCSSpectrumAlternateSign(_autoCrossSpectrum, image);
      fftSpectrumToPolar(image, image);
      fftSwapQuadrants(image, image);
      mask.release();
      break;
    }
    case DISPLAY_FILTER: {
      fftSwapQuadrants(_bandpassFilter, image);
      mask.release();
      break;
    }
    case DISPLAY_INVERSE_CROSS: {
      _inverseCross.copyTo(image);
      mask.release();
      break;
    }

    default:
      break;
  }

  return true;
}

