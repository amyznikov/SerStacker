/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>



template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_REFERENCE_IMAGE, "REFERENCE_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_BLEND_IMAGE, "BLEND_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_SHIFTED_CURRENT_IMAGE, "SHIFTED_CURRENT_IMAGE", "" },


      { c_alpha_test_routine::DISPLAY_CURRENT_SCALED_IMAGE,"CURRENT_SCALED_IMAGE"},
      { c_alpha_test_routine::DISPLAY_REFERENCE_SCALED_IMAGE,"REFERENCE_SCALED_IMAGE"},
      { c_alpha_test_routine::DISPLAY_BLEND_SCALED_IMAGE, "BLEND_SCALED_IMAGE", "" },

      { c_alpha_test_routine::DISPLAY_CROSS_MODULE,"CROSS_MODULE"},
      { c_alpha_test_routine::DISPLAY_CROSS_PHASE,"CROSS_PHASE"},

      { c_alpha_test_routine::DISPLAY_CROSS_pureCross,"pureCross"},
      { c_alpha_test_routine::DISPLAY_CROSS_weightMap,"weightMap"},
      { c_alpha_test_routine::DISPLAY_CROSS_gradX,"gradX"},
      { c_alpha_test_routine::DISPLAY_CROSS_gradY,"gradY"},

      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

/////////////////////////////////////
namespace {

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

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
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

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "downscaleFactor", ctx,  &this_class::downscaleFactor, &this_class::set_downscaleFactor, "");
  ctlbind(ctls, "gsigma", ctx,  &this_class::gsigma, &this_class::set_gsigma, "");
  ctlbind(ctls, "apodizationSize", ctx,  &this_class::apodizationSize, &this_class::set_apodizationSize, "");
  ctlbind(ctls, "updateReference", CTL_CONTEXT(ctx, _updateReferenceImage), "Set checked to set current image as reference");
}


bool c_alpha_test_routine::reinitialize(const cv::Size & expectedFrameSize)
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

  // Find the closest power of two (round mathematically to the nearest)
  // cvRound(std::log2(v)) will select the power that is closest to the target
  // Some limit from below (for example not less than 64 pixels, so that the algorithm does not degenerate)
  // Return the size as 2^powX and 2^powY

  const int downscaledW = cvRound(expectedFrameSize.width / _downscaleFactor);
  const int downscaledH = cvRound(expectedFrameSize.height / _downscaleFactor);
  const int powX = std::max(6, cvCeil(std::log2(downscaledW)));
  const int powY = std::max(6, cvCeil(std::log2(downscaledH)));
  _fftSize = cv::Size(1 << powX, 1 << powY);

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
  return true;
}

bool c_alpha_test_routine::setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  if( _fftSize.empty() ) {
    CF_ERROR("was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( referenceImage.empty() ) {
    CF_ERROR("referenceImage is empty");
    return false;
  }

  if ( referenceImage.channels() == 1 ) {
    referenceImage.getMat().convertTo(_referenceImage, CV_32F);
  }
  else {
    cv::Mat tmp;
    cv::cvtColor(referenceImage, tmp, cv::COLOR_BGR2GRAY);
    tmp.convertTo(_referenceImage, CV_32F);
  }

  if (!referenceMask.empty() && referenceMask.channels() != 1 ) {
    CF_ERROR("referenceMask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;

  const double scale = 1.0 / _downscaleFactor;
  cv::resize(referenceImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  if (!referenceMask.empty()) {
    cv::resize(referenceMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  }

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      _scaledReferenceImage, _scaledReferenceMask, _fftSize,
      _referenceValidSize,
      _referenceCropOffset);

  if (_apodizationSize > 0) {
    applyApodization(_scaledReferenceImage, _scaledReferenceMask,
        _referenceValidSize);
  }

  cv::dft(_scaledReferenceImage, _referenceSpectrum,
      cv::DFT_COMPLEX_OUTPUT);

  return true;
}

bool c_alpha_test_routine::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  if( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( currentImage.empty() ) {
    CF_ERROR("c_phase_correlate: currentImage is empty");
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
    CF_ERROR("c_phase_correlate: currentMask must be single-channel");
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
      cv::DFT_COMPLEX_OUTPUT);

  return true;
}

void c_alpha_test_routine::applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask,
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

bool c_alpha_test_routine::compute(cv::Vec2f & outputTranslation)
{
    if (_currentSpectrum.empty() || _referenceSpectrum.empty()) {
        return false;
    }

    const int rows = _fftSize.height;
    const int cols = _fftSize.width;

    _crossSpectrum.create(_fftSize);
    _crossSpectrum.setTo(cv::Scalar::all(0));
    _gradX.create(_fftSize);
    _gradX.setTo(cv::Scalar::all(0));
    _gradY.create(_fftSize);
    _gradY.setTo(cv::Scalar::all(0));
    _pureCross = cv::Mat2f::zeros(rows, cols);
    _weightMap = cv::Mat1f::zeros(rows, cols);

    const float sigma_pixels = float(_gsigma * std::max(rows, cols));
    const float inv_gsigma2 = -0.5f / (sigma_pixels * sigma_pixels);

    for (int y = 0; y < rows; ++y) {
        const cv::Vec2f* src_curr = _currentSpectrum.ptr<cv::Vec2f>(y);
        const cv::Vec2f* src_ref  = _referenceSpectrum.ptr<cv::Vec2f>(y);
        cv::Vec2f* dst_cross       = _crossSpectrum.ptr<cv::Vec2f>(y);

        float v_freq = (y <= rows / 2) ? (float)y : (float)(rows - y);
        float v2 = v_freq * v_freq;

        for (int x = 0; x < cols; ++x) {
            float a = src_curr[x][0]; float b = src_curr[x][1];
            float c = src_ref[x][0];  float d = src_ref[x][1];

            float re = a * c + b * d;
            float im = b * c - a * d;
            dst_cross[x] = cv::Vec2f(re, im);

            if (y == 0 || x == 0 || y == rows - 1 || x == cols - 1) {
                continue;
            }

            float raw_mag = std::sqrt(re * re + im * im);
            if (raw_mag < 1e-7f) continue;

            float u_freq = (x <= cols / 2) ? (float)x : (float)(cols - x);
            float rho2 = u_freq * u_freq + v2;
            float weight = inv_gsigma2 < 0 ? sqrt(rho2) * std::exp(rho2 * inv_gsigma2) : 1;
            _weightMap(y, x) = weight;

            //_pureCross(y, x) = cv::Vec2f(re / raw_mag, im / raw_mag);
            //_pureCross(y, x) = cv::Vec2f(re , im );
            _pureCross(y, x) = cv::Vec2f(weight * re/ raw_mag , weight * im/ raw_mag );

        }
    }

    cv::Vec2f total_grad_x(0.0f, 0.0f);
    cv::Vec2f total_grad_y(0.0f, 0.0f);

    int max_u = cols / 2;
    int max_v = rows / 2;

    for (int y = 1; y < max_v - 1; ++y) {
        for (int x = 1; x < max_u - 1; ++x) {

            float w = 1; //  _weightMap(y, x);

            cv::Vec2f F_curr_x = _pureCross(y, x);
            cv::Vec2f F_next_x = _pureCross(y, x + 1);

            // F_next * conj(F_curr) -> (A + iB)*(C - iD) = (AC + BD) + i(BC - AD)
            float re_dx = F_next_x[0] * F_curr_x[0] + F_next_x[1] * F_curr_x[1];
            float im_dx = F_next_x[1] * F_curr_x[0] - F_next_x[0] * F_curr_x[1];

            total_grad_x += cv::Vec2f(re_dx * w, im_dx * w);

            cv::Vec2f F_curr_y = _pureCross(y, x);
            cv::Vec2f F_next_y = _pureCross(y + 1, x);
            float re_dy = F_next_y[0] * F_curr_y[0] + F_next_y[1] * F_curr_y[1];
            float im_dy = F_next_y[1] * F_curr_y[0] - F_next_y[0] * F_curr_y[1];

            total_grad_y += cv::Vec2f(re_dy * w, im_dy * w);

            _gradX(y, x) = cv::Vec2f(re_dx, im_dx);
            _gradY(y, x) = cv::Vec2f(re_dy, im_dy);

        }
    }

    float final_delta_x = std::atan2(total_grad_x[1], total_grad_x[0]);
    float final_delta_y = std::atan2(total_grad_y[1], total_grad_y[0]);

    float Tx = (final_delta_x * cols) / (2.0f * CV_PI);
    float Ty = (final_delta_y * rows) / (2.0f * CV_PI);

    outputTranslation[0] = Tx * (float)_downscaleFactor;
    outputTranslation[1] = Ty * (float)_downscaleFactor;

    return true;
}


static void shiftImage(cv::InputArray src, cv::OutputArray dst, const cv::Vec2f& translation)
{
    float data[6] = {
        1.0f, 0.0f, translation[0],
        0.0f, 1.0f, translation[1]
    };
    cv::Mat M(2, 3, CV_32FC1, data);
    cv::warpAffine(src, dst, M, src.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !_initialized && !reinitialize(image.size()) )  {
    CF_ERROR("reinitialize() fails");
    return false;
  }

  if ( _referenceImage.empty() || _updateReferenceImage ) {
    if ( !setReferenceImage(image, mask) ) {
      CF_ERROR("setReferenceImage() fails");
      return false;
    }
  }

  if ( !_referenceImage.empty() ) {
    if ( !setCurrentImage(image, mask) ) {
      CF_ERROR("setCurrentImage() fails");
      return false;
    }

    cv::Vec2f T;
    if ( !compute(T) ) {
      CF_ERROR("compute() fails");
    }
    else {
      CF_DEBUG("\ncompute: Tx = %g Ty = %g T=%g\n"
          "_downscaleFactor=%g\n"
          "_currentValidSize = %dx%d\n"
          "_referenceValidSize = %dx%d\n"
          "_currentCropOffset.x = %d _currentCropOffset.y = %d\n"
          "_referenceCropOffset.x = %d _referenceCropOffset.y = %d\n"
          "\n",
          T[0], T[1], std::hypot(T[0], T[1]),
          _downscaleFactor,
          _currentValidSize.width, _currentValidSize.height,
          _referenceValidSize.width, _referenceValidSize.height,
          _currentCropOffset.x, _currentCropOffset.y,
          _referenceCropOffset.x, _referenceCropOffset.y);
    }

    switch (_display) {
      case DISPLAY_CURRENT_IMAGE: {
        _currentImage.copyTo(image);
        mask.release();
        break;
      }
      case DISPLAY_REFERENCE_IMAGE: {
        _referenceImage.copyTo(image);
        mask.release();
        break;
      }
      case DISPLAY_CURRENT_SCALED_IMAGE: {
        _scaledCurrentImage.copyTo(image);
        _scaledCurrentMask.copyTo(mask);
        break;
      }
      case DISPLAY_REFERENCE_SCALED_IMAGE: {
        _scaledReferenceImage.copyTo(image);
        _scaledReferenceMask.copyTo(mask);
        break;
      }

      case DISPLAY_SHIFTED_CURRENT_IMAGE : {
        shiftImage(_currentImage, image, T);
        mask.release();
        break;
      }

      case DISPLAY_BLEND_IMAGE: {
        if ( !_currentImage.empty() && !_referenceImage.empty() ) {
          cv::addWeighted(_currentImage, 0.5, _referenceImage, 0.5, 0, image);
        }
        else if ( !_currentImage.empty() ) {
          _currentImage.copyTo(image);
        }
        else if ( !_referenceImage.empty() ) {
          _referenceImage.copyTo(image);
        }
        else {
          // do nothing, keep ioutput image as is
        }

        mask.release();
        break;
      }
      case DISPLAY_BLEND_SCALED_IMAGE: {
        if ( !_scaledCurrentImage.empty() && !_scaledReferenceImage.empty() ) {
          cv::addWeighted(_scaledCurrentImage, 0.5, _scaledReferenceImage, 0.5, 0, image);
        }
        else if ( !_scaledCurrentImage.empty() ) {
          _scaledCurrentImage.copyTo(image);
        }
        else if ( !_scaledReferenceImage.empty() ) {
          _scaledReferenceImage.copyTo(image);
        }
        else {
          // do nothing, keep ioutput image as is
        }
        mask.release();
        break;
      }

      case DISPLAY_CROSS_MODULE: {
        mask.release();
        if ( _crossSpectrum.empty() ) {
          image.release();
        }
        else {
          fftSpectrumModule(_crossSpectrum, image);
        }

        break;
      }
      case DISPLAY_CROSS_PHASE: {
        mask.release();
        if ( _crossSpectrum.empty() ) {
          image.release();
        }
        else {
          // will computed as direct atan2() in radians
          fftSpectrumPhase(_crossSpectrum, image);
        }
        break;
      }
      case DISPLAY_CROSS_pureCross: {
        _pureCross.copyTo(image);
        mask.release();
        break;
      }
      case DISPLAY_CROSS_weightMap: {
        _weightMap.copyTo(image);
        mask.release();
        break;
      }
      case DISPLAY_CROSS_gradX: {
        _gradX.copyTo(image);
        mask.release();
        break;
      }
      case DISPLAY_CROSS_gradY: {
        _gradY.copyTo(image);
        mask.release();
        break;
      }

      default:
        break;
    }
  }

  return true;
}


