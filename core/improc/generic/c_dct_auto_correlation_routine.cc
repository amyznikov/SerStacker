/*
 * c_dct_auto_correlation_routine.cc
 *
 *  Created on: Oct 13, 2026
 *      Author: amyznikov
 */

#include "c_dct_auto_correlation_routine.h"
#include <core/proc/run-loop.h>
#include <core/proc/fft.h>

template<>
const c_enum_member* members_of<c_dct_auto_correlation_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_dct_auto_correlation_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_dct_auto_correlation_routine::DISPLAY_CURRENT_SCALED_IMAGE, "SCALED_IMAGE" },
      { c_dct_auto_correlation_routine::DISPLAY_CURRENT_SPECTRUM_DCT, "IMAGE_SPECTRUM_DCT" },
      { c_dct_auto_correlation_routine::DISPLAY_CORRELATION_MAP, "CORRELATION_MAP" },
      { c_dct_auto_correlation_routine::DISPLAY_FILTER, "FILTER" },
      { c_dct_auto_correlation_routine::DISPLAY_CORRELATION_MAP }
  };
  return members;
}

namespace {

static int getOptimalDCTSizeDown(int size)
{
  if( size % 2 != 0 ) {
    size--;
  }
  int sopt = cv::getOptimalDFTSize(size);
  while (sopt > 0 && sopt > size) {
    sopt = cv::getOptimalDFTSize(--size);
  }
  if( sopt % 2 != 0 ) {
    sopt--;
  }
  return sopt;
}

static cv::Size computeDCTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor)
{
  const int downscaledW = getOptimalDCTSizeDown(cvRound(expectedFrameSize.width / downscaleFactor));
  const int downscaledH = getOptimalDCTSizeDown(cvRound(expectedFrameSize.height / downscaleFactor));
  return cv::Size(std::max(4, downscaledW), std::max(4, downscaledH));
}

// Frame padding. Fill empty areas strictly with zeros (symmetric DCT extension will handle the rest).
static void packScaledImageForDCT(cv::InputArray srcImage, cv::InputArray srcMask,
    cv::Mat1f & outImage, cv::Mat1b & outMask,
    const cv::Size & dctSize,
    cv::Size & outputValidSize,
    cv::Point & cropOffset)
{
  cv::Mat smallImage, smallMask;

  if( srcImage.cols() <= dctSize.width && srcImage.rows() <= dctSize.height ) {
    cropOffset = cv::Point(0, 0);
    smallImage = srcImage.getMat();
    smallMask = srcMask.getMat();
    outputValidSize = smallImage.size();
  }
  else {
    const int cropW = std::min(srcImage.cols(), dctSize.width);
    const int cropH = std::min(srcImage.rows(), dctSize.height);
    cropOffset.x = (srcImage.cols() - cropW) / 2;
    cropOffset.y = (srcImage.rows() - cropH) / 2;
    const cv::Rect roi(cropOffset.x, cropOffset.y, cropW, cropH);
    smallImage = srcImage.getMat()(roi);
    if( !srcMask.empty() ) {
      smallMask = srcMask.getMat()(roi);
    }
    outputValidSize = cv::Size(cropW, cropH);
  }

  const int bottom = dctSize.height - smallImage.rows;
  const int right = dctSize.width - smallImage.cols;

  if( bottom < 1 && right < 1 ) {
    if( smallImage.depth() == CV_32F ) {
      outImage = std::move(smallImage);
    }
    else {
      smallImage.convertTo(outImage, CV_32F);
    }
    if( smallMask.empty() ) {
      outMask = cv::Mat1b(dctSize, 255);
    }
    else if( smallMask.depth() == CV_8U ) {
      outMask = std::move(smallMask);
    }
    else {
      cv::compare(smallMask, 0, outMask, cv::CMP_GT);
    }
  }
  else {
    const cv::Rect validROI(0, 0, outputValidSize.width, outputValidSize.height);

    outImage.create(dctSize), outImage.setTo(0);
    outMask.create(dctSize), outMask.setTo(0);

    if( smallImage.depth() == CV_32F ) {
      smallImage.copyTo(outImage(validROI));
    }
    else {
      smallImage.convertTo(outImage(validROI), CV_32F);
    }

    if( smallMask.empty() ) {
      outMask(validROI).setTo(255);
    }
    else if( smallMask.depth() == CV_8U ) {
      smallMask.copyTo(outMask(validROI));
    }
    else {
      cv::compare(smallMask, 0, outMask(validROI), cv::CMP_GT);
    }
  }
}

} // namespace

bool c_dct_auto_correlation_routine::ensureInitialized(const cv::Size & expectedFrameSize)
{
  if( expectedFrameSize.empty() ) {
    CF_ERROR("BAD expectedFrameSize specified : %dx%d", expectedFrameSize.width, expectedFrameSize.height);
    return false;
  }

  if( _downscaleFactor < 1 ) {
    CF_ERROR("downscale_factor specified : %g. Must be >= 1", _downscaleFactor);
    return false;
  }

  const cv::Size dctSize = computeDCTPackSize(expectedFrameSize, _downscaleFactor);
  if( !_initialized || dctSize != _dctSize ) {
    _dctSize = dctSize;

    if( !_currentImage.empty() ) {
      generateBandpassFilter();
    }

    _initialized = true;
  }

  return _initialized;
}

void c_dct_auto_correlation_routine::generateBandpassFilter()
{
  // In DCT space (Type II), the DC component is located STRICTLY at point (0,0).
  // All frequencies increase linearly from 0 to (cols-1) and (rows-1). No reversals or sign alternations!
  INSTRUMENT_REGION("");

  _bandpassFilter.create(_dctSize);

  const uint8_t * filter_base = _bandpassFilter.ptr();
  const size_t filter_stride = _bandpassFilter.step;

  const int cols = _dctSize.width;
  const int rows = _dctSize.height;

  if( _gsigma <= 0 ) {
    _bandpassFilter.setTo(1.0f);
  }
  else {
    // Unlike the DFT, the DCT frequency grid is one-sided (cosine): frequencies range from 0 to N-1.
    const double ilambda_x = (M_PI * _gsigma) / cols;
    const double ilambda_y = (M_PI * _gsigma) / rows;
    const float ilambda_x2 = float(ilambda_x * ilambda_x);
    const float ilambda_y2 = float(ilambda_y * ilambda_y);

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);
        const float v2 = float(y * y) * ilambda_y2;

        for( int x = 0; x < cols; ++x ) {
          const float u2 = float(x * x) * ilambda_x2;
          const float rho2 = u2 + v2;
          fltp[x] = rho2 * std::exp(-rho2);
        }
      }
    });
  }

  cv::multiply(_bandpassFilter, 1. / cv::norm(_bandpassFilter, cv::NORM_L1), _bandpassFilter);
}

bool c_dct_auto_correlation_routine::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  if( _dctSize.empty() ) {
    CF_ERROR("c_dct_auto_correlation: was not properly initialized, _dctSize is empty");
    return false;
  }

  if( currentImage.empty() ) {
    CF_ERROR("c_dct_auto_correlation: currentImage is empty");
    return false;
  }

  if( currentImage.channels() == 1 ) {
    currentImage.getMat().convertTo(_currentImage, CV_32F);
  }
  else {
    cv::Mat tmp;
    cv::cvtColor(currentImage, tmp, cv::COLOR_BGR2GRAY);
    tmp.convertTo(_currentImage, CV_32F);
  }

  if( !currentMask.empty() && currentMask.channels() != 1 ) {
    CF_ERROR("c_dct_auto_correlation: currentMask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;
  const double scale = 1.0 / _downscaleFactor;
  cv::resize(_currentImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  if( !currentMask.empty() ) {
    cv::resize(currentMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  }

  packScaledImageForDCT(smallImage, smallMask,
      _scaledCurrentImage, _scaledCurrentMask, _dctSize,
      _currentValidSize,
      _currentCropOffset);

  cv::dct(_scaledCurrentImage, _currentSpectrum);

  return true;
}

void c_dct_auto_correlation_routine::computeCorrelationMap()
{
  if( _bandpassFilter.empty() ) {
    generateBandpassFilter();
  }

  // In the DCT, the spectrum is purely real.
  // The auto-power spectrum is simply the element-wise square of the frame spectrum,
  // multiplied by a filter. No complex conjugation is required.
  cv::multiply(_currentSpectrum, _currentSpectrum, _autoCrossSpectrum);
  cv::multiply(_autoCrossSpectrum, _bandpassFilter, _autoCrossSpectrum);

  // Energy of the filtered cross-spectrum for peak normalization
  _crossSpectrumEnergy = cv::norm(_autoCrossSpectrum, cv::NORM_L2SQR);

  cv::idct(_autoCrossSpectrum, _autoCorrelationMap);
}

bool c_dct_auto_correlation_routine::analyzeSpotGeometry()
{
  const int rows = _autoCorrelationMap.rows;
  const int cols = _autoCorrelationMap.cols;

  // Due to the nature of DCT mirroring, the autocorrelation peak is always located strictly at the point (0,0).
  const int cx = 0;
  const int cy = 0;

  const double sgsigma = _gsigma / _downscaleFactor;
  const double Rspot = 0.5 * sgsigma;

  const int R = std::max(3, cvCeil(Rspot));

  // ROI at the top-left corner (0,0), without going out of bounds
  const cv::Rect ROI = cv::Rect(0, 0, R + 1, R + 1) & cv::Rect(0, 0, cols, rows);
  const cv::Mat1f roi = _autoCorrelationMap(ROI);

  cv::Mat1f croi;
  cv::subtract(roi, cv::mean(roi), croi);
  cv::max(croi, 0, croi);
  const cv::Moments m = cv::moments(croi, false);

  static constexpr float safety_thresh = std::numeric_limits<float>::min();

  _spot.radius = R;
  _spot.peak = 1024.0 * _autoCorrelationMap(cy, cx) / std::sqrt(_dctSize.area() * _crossSpectrumEnergy);

  if( m.m00 > safety_thresh ) {
    const double mu20 = m.mu20 / m.m00;
    const double mu02 = m.mu02 / m.m00;
    const double mu11 = m.mu11 / m.m00;

    const double delta = mu20 - mu02;
    const double sqrt_term = std::sqrt(delta * delta + 4.0 * mu11 * mu11);
    const double lambda1 = 0.5 * (mu20 + mu02 + sqrt_term);
    const double lambda2 = 0.5 * (mu20 + mu02 - sqrt_term);

    const double sigma_to_fwhm = 2.35482004503;
    _spot.fwhm_x = std::sqrt(std::max(0.0, lambda1)) * sigma_to_fwhm;
    _spot.fwhm_y = std::sqrt(std::max(0.0, lambda2)) * sigma_to_fwhm;

    if( std::abs(delta) > safety_thresh || std::abs(mu11) > safety_thresh ) {
      _spot.angle = 0.5 * std::atan2(2.0 * mu11, delta) * (180.0 / CV_PI);
    }
    else {
      _spot.angle = 0.0;
    }

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

bool c_dct_auto_correlation_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _downscaleFactor);
    SERIALIZE_OPTION(settings, save, *this, _gsigma);
    return true;
  }
  return false;
}

void c_dct_auto_correlation_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "downscaleFactor", ctx, &this_class::downscaleFactor, &this_class::set_downscaleFactor, "");
  ctlbind(ctls, "gsigma", ctx, &this_class::gsigma, &this_class::set_gsigma, "");
  ctlbind(ctls, "print debug", CTL_CONTEXT(ctx, _printDebug), "");
}

bool c_dct_auto_correlation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !ensureInitialized(image.size()) ) {
    CF_ERROR("reinitialize() fails");
    return false;
  }
  if( !setCurrentImage(image, mask) ) {
    CF_ERROR("setCurrentImage() fails");
    return false;
  }

  computeCorrelationMap();
  if( _printDebug ) {
    if( !analyzeSpotGeometry() ) {
      CF_ERROR("analyzeSpotGeometry() fails");
    }
    else {
      CF_DEBUG("\nDCT Spot: R=%3g peak = %8.3f fwhm = %7.3f x%7.3f angle = %7.3f ecc = %6.3f\n",
          _spot.radius, _spot.peak, _spot.fwhm_x, _spot.fwhm_y, _spot.angle, _spot.eccentricity);
    }
  }

  switch (_display) {
    case DISPLAY_CORRELATION_MAP: {
      const double combinedScale = 1024.0 / std::sqrt(_dctSize.area() * _crossSpectrumEnergy);
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
    case DISPLAY_CURRENT_SPECTRUM_DCT: {
      _currentSpectrum.copyTo(image);
      mask.release();
      break;
    }
    case DISPLAY_FILTER: {
      _bandpassFilter.copyTo(image);
      cv::normalize(image, image, 0, 1, cv::NORM_MINMAX);
      mask.release();
      break;
    }
    default:
      break;
  }
  return true;
}
