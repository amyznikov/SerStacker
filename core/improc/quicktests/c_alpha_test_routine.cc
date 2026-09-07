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
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>



template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_PREVIOUS_IMAGE, "PREVIOUS_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_IFFT, "IFFT", "" },
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

void c_phase_correlate::scaleAndPadToTargetSize(cv::InputArray srcImage, cv::InputArray srcMask,
    cv::Mat & dstImage, cv::Mat & dstMask,
    const cv::Size & targetSize, double scaleFactor)
{
  const cv::Mat img = srcImage.getMat();
  const cv::Mat msk = srcMask.getMat();

  const cv::Size scaledSize(cvRound(img.cols * scaleFactor),
    cvRound(img.rows * scaleFactor));

  cv::Mat scaledImg;
  cv::resize(img, scaledImg, scaledSize, 0, 0, cv::INTER_AREA);

  cv::Mat scaledMsk;
  if ( msk.empty()) {
    scaledMsk = cv::Mat(scaledSize, CV_8UC1, cv::Scalar(255));
  }
  else {
    cv::Mat binaryMask;
    cv::compare(msk, 0, binaryMask, cv::CMP_GT);
    static const cv::Mat1b SE(3, 3, 255);
    cv::erode(binaryMask, binaryMask, SE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    cv::resize(binaryMask, scaledMsk, scaledSize, 0, 0, cv::INTER_NEAREST);
  }

  int deltaW = targetSize.width - scaledSize.width;
  int deltaH = targetSize.height - scaledSize.height;
  int top    = std::max(0, deltaH / 2);
  int bottom = std::max(0, deltaH - top);
  int left   = std::max(0, deltaW / 2);
  int right  = std::max(0, deltaW - left);

  if (top > 0 || bottom > 0 || left > 0 || right > 0) {
    cv::copyMakeBorder(scaledImg, dstImage, top, bottom, left, right, cv::BORDER_REFLECT_101);
    cv::copyMakeBorder(scaledMsk, dstMask, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0));
  }
  else if (deltaW < 0 || deltaH < 0) {
    const int cropX = std::max(0, -deltaW / 2);
    const int cropY = std::max(0, -deltaH / 2);
    dstImage = scaledImg(cv::Rect(cropX, cropY, targetSize.width, targetSize.height)).clone();
    dstMask  = scaledMsk(cv::Rect(cropX, cropY, targetSize.width, targetSize.height)).clone();
  }
  else {
    dstImage = scaledImg;
    dstMask  = scaledMsk;
  }
}

void c_phase_correlate::createApodizationWindow(cv::InputArray mask, cv::Mat1f & outputWindow, int kradius)
{
  if( mask.empty() || kradius <= 0 ) {
    outputWindow = cv::Mat1f::ones(mask.size());
    return;
  }

  cv::Mat inputMask = mask.getMat();
  cv::Mat blurredMask;

  int scale = 1;
  if( kradius > 64 )
    scale = 8;
  else if( kradius > 16 )
    scale = 4;
  else if( kradius > 4 )
    scale = 2;

  if( scale > 1 ) {
    cv::Mat smallMask;
    cv::resize(inputMask, smallMask, cv::Size(inputMask.cols / scale, inputMask.rows / scale), 0, 0, cv::INTER_NEAREST);

    cv::Mat distMap;
    cv::distanceTransform(smallMask, distMap, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    float smallRadius = float(kradius) / scale;
    distMap /= smallRadius;
    cv::threshold(distMap, distMap, 1.0, 1.0, cv::THRESH_TRUNC);

    cv::resize(distMap, blurredMask, inputMask.size(), 0, 0, cv::INTER_LINEAR);
  }
  else {
    cv::Mat distMap;
    cv::distanceTransform(inputMask, distMap, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    distMap /= float(kradius);
    cv::threshold(distMap, distMap, 1.0, 1.0, cv::THRESH_TRUNC);
    blurredMask = distMap;
  }

  int ksize = 2 * scale + 1;
  if( ksize > 1 ) {
    cv::boxFilter(blurredMask, blurredMask, CV_32F, cv::Size(ksize, ksize));
  }

  outputWindow = blurredMask;
}

cv::Point2f c_phase_correlate::findSubpixelCentroid(const cv::Mat1f & idctResult, cv::Point & outPeakLoc)
{
  const int rows = idctResult.rows;
  const int cols = idctResult.cols;

  cv::minMaxLoc(idctResult, nullptr, nullptr, nullptr, &outPeakLoc);

  double sumMass = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;

  const int r = 2; // 5х5

  for( int dy = -r; dy <= r; ++dy ) {
    int sampleY = outPeakLoc.y + dy;
    if( sampleY < 0 ) {
      sampleY += rows;
    }
    if( sampleY >= rows ) {
      sampleY -= rows;
    }

    for( int dx = -r; dx <= r; ++dx ) {
      int sampleX = outPeakLoc.x + dx;
      if( sampleX < 0 ) {
        sampleX += cols;
      }
      if( sampleX >= cols ) {
        sampleX -= cols;
      }

      float mass = std::max(0.0f, idctResult(sampleY, sampleX));
      double virtualX = static_cast<double>(outPeakLoc.x + dx);
      double virtualY = static_cast<double>(outPeakLoc.y + dy);
      sumMass += mass;
      sumX += virtualX * mass;
      sumY += virtualY * mass;
    }
  }

  double subX = (sumMass > 0.0) ? (sumX / sumMass) : static_cast<double>(outPeakLoc.x);
  double subY = (sumMass > 0.0) ? (sumY / sumMass) : static_cast<double>(outPeakLoc.y);

  return cv::Point2f(float(subX), float(subY));
}

double c_phase_correlate::computeCorrelationMap(cv::InputArray image1, cv::InputArray image2,
    cv::OutputArray outputCcorrelationMap, double _gsigma) const
{
  // Assume input args are valid CV_32FC1 matrices of valid (power of 2) size each,
  // so no any padding is required, assume all the padding is responsibility of caller code

  const cv::Mat1f src1 = image1.getMat();
  const cv::Mat1f src2 = image2.getMat();

  cv::Mat1f spec1, spec2;
  cv::dft(src1, spec1, cv::DFT_REAL_OUTPUT);
  cv::dft(src2, spec2, cv::DFT_REAL_OUTPUT);

  const int rows = spec1.rows;
  const int cols = spec1.cols;
  const bool is_even = (cols % 2 == 0);
  const float gsigma = float(_gsigma);
  const float norm_factor = float(1.64872127 / _gsigma);
  const float inv_cols = 1.0f / (float)cols;
  const float inv_rows = 1.0f / (float)rows;

  cv::Mat1f cross(spec1.size());

  const uint8_t * spec1_base = spec1.ptr();
  const size_t spec1_stride = spec1.step;

  const uint8_t * spec2_base = spec2.ptr();
  const size_t spec2_stride = spec2.step;

  uint8_t * cross_base = cross.ptr();
  const size_t cross_stride = cross.step;

  // Constants for the incremental Gaussian step
  // u = fx / cols.
  // In the exponent:
  // -0.5 * u^2 / gsigma^2 = -0.5 * fx^2 / (cols^2 * gsigma^2)
  const float alpha = -0.5f / (gsigma * gsigma * (float)cols * (float)cols);
  const float exp_alpha = std::exp(alpha);
  const float exp_alpha2 = std::exp(2.0f * alpha);

  std::atomic<float> total_energy(0.0f);

  parallel_for(0, rows, [=, &total_energy](const auto& range) {
    float local_energy = 0.0f;

    for( int y = rbegin(range); y < rend(range); ++y ) {
      const float * srcp1 =  (const float * )(spec1_base + y * spec1_stride);
      const float * srcp2 =  (const float * )(spec2_base + y * spec2_stride);
      float * __restrict dstp = (float *)(cross_base + y * cross_stride);

      // Optimization for vertical sign and frequency
      const float sign_y = (y % 2 == 0) ? 1.0f : -1.0f;
      const float v = (y > rows / 2) ? (float)(rows - y) * inv_rows : (float)y * inv_rows;
      const float v_sq = v * v;

      // Precomputed vertical part of the Gaussian exponential for the current row
      const float weight_v = std::exp(-0.5f * v_sq / (gsigma * gsigma));

      dstp[0] = 0.0f;

      // Incremental horizontal Gaussian calculation (for fx = 1)
      // Instead of calling std::exp on each iteration, there will simply be a multiplication
      float g_u = std::exp(alpha);
      float g_step = std::exp(3.0f * alpha);

      // Sign for fx = 1 (since fx=1 is even? No, 1 is odd, so (1 % 2 == 0) -> false -> -1.0f)
      // shift_sign for fx=1 will be equal to -sign_y.
      // The sign will simply change with each iteration.
      float current_shift_sign = -sign_y;

      const int max_complex_idx = is_even ? (cols - 2) : (cols - 1);
      for( int x = 1; x <= max_complex_idx; x += 2 ) {
        const int fx = (x + 1) / 2;

        // Instead of u = fx * inv_cols and heavy std::sqrt(u*u + v*v)
        const float u = (float)fx * inv_cols;
        const float rho = std::sqrt(u * u + v_sq);

        // weight = norm_factor * rho * exp(-0.5*v^2/g^2) * exp(-0.5*u^2/g^2)
        const float weight = norm_factor * rho * weight_v * g_u;

        // Incremental Gaussian step for the next iteration of fx
        g_u *= g_step;
        g_step *= exp_alpha2;

        const float a = srcp1[x];
        const float b = srcp1[x + 1];
        const float c = srcp2[x];
        const float d = srcp2[x + 1];

        const float re = a * c + b * d;
        const float im = b * c - a * d;
        const float magnitude = std::sqrt(re * re + im * im);

        if( magnitude > 0 ) {
          const float factor = weight * current_shift_sign / magnitude;
          dstp[x] = re * factor;
          dstp[x + 1] = im * factor;
          local_energy += 2 * weight;
        }
        else {
          dstp[x] = 0.0f;
          dstp[x + 1] = 0.0f;
        }

        // Swap the sign for the next fx (alternating 1 and -1)
        current_shift_sign = -current_shift_sign;
      }

      if( is_even ) {
        dstp[cols - 1] = 0.0f;
      }
    }

    float current = total_energy.load(std::memory_order_relaxed);
    while (!total_energy.compare_exchange_weak(current, current + local_energy,
        std::memory_order_relaxed));
  });

  const double total_filter_energy =
      total_energy.load();

  cv::idft(cross, outputCcorrelationMap,
      cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);

  return total_filter_energy;
}

double c_phase_correlate::compute(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Vec2f * outputTranslation)
{
  if (currentImage.empty() || referenceImage.empty()) {
    return -1.0;
  }

  cv::Mat inputCurrent = currentImage.getMat();
  cv::Mat inputReference = referenceImage.getMat();

  const cv::Size targetSize = getOptimalPhaseCorrelationSize(inputCurrent.size());
  if (targetSize != _cachedSize) {
    _cachedSize = targetSize;
    _cachedScaleFactor = double(targetSize.width) / inputCurrent.cols;
  }

  const int scaledRadius = std::max(1, cvRound(_opts.apodization_radius * _cachedScaleFactor));

  scaleAndPadToTargetSize(currentImage, currentMask, _scaledImg1, _scaledMsk1, _cachedSize, _cachedScaleFactor);
  scaleAndPadToTargetSize(referenceImage, referenceMask, _scaledImg2, _scaledMsk2, _cachedSize, _cachedScaleFactor);

  if( _opts.apodization_radius < 1 ) {
    _maskedImg1 = _scaledImg1;
    _maskedImg2 = _scaledImg2;
  }
  else {
    createApodizationWindow(_scaledMsk1, _window1, scaledRadius);
    createApodizationWindow(_scaledMsk2, _window2, scaledRadius);
    cv::multiply(_scaledImg1, _window1, _maskedImg1);
    cv::multiply(_scaledImg2, _window2, _maskedImg2);
  }

  const double total_filter_energy =
      computeCorrelationMap(_maskedImg1, _maskedImg2, correlationMap,
          _opts.gsigma);

  cv::Point peakLoc;
  const cv::Point2f scaledTranslation = findSubpixelCentroid(correlationMap, peakLoc);
  const double rawPeak = correlationMap(peakLoc.y, peakLoc.x);
  const double normalizedResponse = (total_filter_energy > 0) ? (rawPeak / total_filter_energy) : 0.0;
  const double correlationScore = normalizedResponse * correlationMap.size().area();

  if( outputTranslation ) {
    (*outputTranslation)[0] = (scaledTranslation.x - correlationMap.cols / 2) * float(1.0 / _cachedScaleFactor);
    (*outputTranslation)[1] = (scaledTranslation.y - correlationMap.rows / 2) * float(1.0 / _cachedScaleFactor);
  }

  return correlationScore;
}

/////////////////////////////////////
namespace {
} // namespace

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, pc._opts, apodization_radius);
    SERIALIZE_OPTION(settings, save, pc._opts, gsigma);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "ApodizationRadius", CTL_CONTEXT(ctx, pc._opts.apodization_radius), "");
  ctlbind(ctls, "gsigma", CTL_CONTEXT(ctx, pc._opts.gsigma), "");

  ctlbind(ctls, "updatePrevious", CTL_CONTEXT(ctx, _updatePreviousImage), "Set checked to update prevImage");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat currentImage, currentMask;

  currentImage = image.getMat();
  currentMask = mask.getMat();

  if ( !prevImage.empty() ) {

    CF_DEBUG("Call compute()");

    cv::Vec2f Translation;

    const double score =
        pc.compute(currentImage, currentMask, prevImage, prevMask,
            &Translation);

    CF_DEBUG("compute: score: %g T: x=%g y=%g", score, Translation[0], Translation[1]);

    switch(_display)
    {
      case DISPLAY_CURRENT_IMAGE:
        break;
      case DISPLAY_PREVIOUS_IMAGE:
        prevImage.copyTo(image);
        prevMask.copyTo(mask);
        break;
      case DISPLAY_IFFT:
        pc.correlationMap.copyTo(image);
        mask.release();
        break;
    }
  }

  if ( _updatePreviousImage ) {
    currentImage.copyTo(prevImage);
    currentMask.copyTo(prevMask);
  }

  return true;
}

