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
      { c_alpha_test_routine::DISPLAY_DCT_CURRENT, "DCT_CURRENT", "" },
      { c_alpha_test_routine::DISPLAY_DCT_PREVIOUS, "DCT_PREVIOUS", "" },
      { c_alpha_test_routine::DISPLAY_DCT_CROSS, "DCT_CROSS", "" },
      { c_alpha_test_routine::DISPLAY_IDCT, "IDCT", "" },
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

//template<>
//const c_enum_member * members_of<c_alpha_test_routine::ResizeMode>()
//{
//  static const c_enum_member members[] = {
//      {c_alpha_test_routine::ResizeModeKeep, "KEEP", ""},
//      {c_alpha_test_routine::ResizeModeAdjust, "ADJUST", ""},
//      {c_alpha_test_routine::ResizeModeCropVisible, "CropVisible", ""},
//      {c_alpha_test_routine::ResizeModeCropVisible},
//  };
//
//  return members;
//}


namespace {

static void createApodizationWindowFromMask(cv::InputArray mask, cv::Mat1f & outputWindow, int kradius)
{
  if (mask.empty() || kradius <= 0) {
    outputWindow = cv::Mat1f::ones(mask.size());
    return;
  }

  cv::Mat inputMask = mask.getMat();

  int scale = 1;
  if (kradius > 64)      scale = 8;
  else if (kradius > 16) scale = 4;
  else if (kradius > 4)  scale = 2;

  cv::Mat smallMask;
  if (scale > 1) {
    cv::resize(inputMask, smallMask, cv::Size(inputMask.cols / scale, inputMask.rows / scale), 0, 0, cv::INTER_NEAREST);
  } else {
    smallMask = inputMask;
  }

  cv::Mat distMap;
  cv::distanceTransform(smallMask, distMap, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  float smallRadius = float(kradius) / scale;
  distMap /= smallRadius;
  cv::threshold(distMap, distMap, 1.0, 1.0, cv::THRESH_TRUNC);

  if (scale > 1) {
    cv::resize(distMap, outputWindow, inputMask.size(), 0, 0, cv::INTER_LINEAR);
  }
  else {
    outputWindow = distMap;
  }
}

static cv::Point2f findSubpixelShiftDCT(const cv::Mat1f & idctCross)
{
  const int rows = idctCross.rows;
  const int cols = idctCross.cols;

  double maxVal;
  cv::Point maxLoc;
  cv::minMaxLoc(idctCross, nullptr, &maxVal, nullptr, &maxLoc);

  double sumMass = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;

  const int r = 2;

  for (int dy = -r; dy <= r; ++dy) {
    int sampleY = maxLoc.y + dy;
    if (sampleY < 0)     {
      sampleY += rows;
    }
    if (sampleY >= rows) {
      sampleY -= rows;
    }

    for (int dx = -r; dx <= r; ++dx) {
      int sampleX = maxLoc.x + dx;
      if (sampleX < 0)     {
        sampleX += cols;
      }
      if (sampleX >= cols) {
        sampleX -= cols;
      }

      float mass = std::max(0.0f, idctCross(sampleY, sampleX));
      double virtualX = static_cast<double>(maxLoc.x + dx);
      double virtualY = static_cast<double>(maxLoc.y + dy);

      sumMass += mass;
      sumX += virtualX * mass;
      sumY += virtualY * mass;
    }
  }

  double subX = (sumMass > 0.0) ? (sumX / sumMass) : static_cast<double>(maxLoc.x);
  double subY = (sumMass > 0.0) ? (sumY / sumMass) : static_cast<double>(maxLoc.y);
  if (subX > cols / 2.0) {
    subX -= cols;
  }
  if (subY > rows / 2.0) {
    subY -= rows;
  }

  return cv::Point2f(static_cast<float>(subX), static_cast<float>(subY));
}

static void compute_gradient(cv::InputArray src, cv::Mat& dstGradient,
    cv::InputArray mask, bool invertedMask = false)
{
  // 4th order derivative vector (5x1)
  // 2nd order smoothing  vector (3x1)
  static const cv::Matx<float, 5, 1> d5( +1.f/12.f, -2.f/3.f, 0.f, +2.f/3.f, -1.f/12.f );
  static const cv::Matx<float, 3, 1> s3( 0.25f, 0.5f, 0.25f );

  cv::Mat gx, gy;

  parallel_invoke(
      [&]() {
        cv::sepFilter2D(src, gx, CV_32F, d5, s3, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      },
      [&]() {
        cv::sepFilter2D(src, gy, CV_32F, s3, d5, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      });

  cv::magnitude(gx, gy, dstGradient);
  if ( !mask.empty() ) {
    dstGradient.setTo(0, invertedMask ? mask.getMat() : ~mask.getMat());
  }
}

} // namespace

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "Differentiate", CTL_CONTEXT(ctx, _applyDifferentiate), "");
  ctlbind(ctls, "applyRAMP", CTL_CONTEXT(ctx, _applyRAMP), "Set checked to apply RAMP filter to dct cross");
  ctlbind(ctls, "applyMaskApodization", CTL_CONTEXT(ctx, _applyMaskApodization), "");
  ctlbind(ctls, "maskApodizationRadius", CTL_CONTEXT(ctx, _maskApodizationKernelRadius), "");
  ctlbind(ctls, "updatePrevious", CTL_CONTEXT(ctx, _updatePrevious), "Set checked to update prevImage");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat currentImage, currentMask, currentGradientImage, prevGradientImage;
  cv::Mat dctCurrent, dctPrevious, dctCross, idctCross;
  cv::Mat maskedCurrentImage, maskedPreviousImage;
  cv::Mat1f window;
  cv::Point2f peakLocation;

  cv::Mat inputImage = image.getMat();
  cv::Mat inputMask = mask.getMat();

  const cv::Size targetSize = getOptimalPhaseCorrelationSize(inputImage.size());
  const double scaleX = double(targetSize.width) / inputImage.cols;
  const int scaledRadius = std::max(1, cvRound(_maskApodizationKernelRadius * scaleX));

  CF_DEBUG("Call resize(inputImage)");
  cv::resize(inputImage, currentImage, targetSize, 0, 0, cv::INTER_AREA);

  if ( !inputMask.empty() ) {
    CF_DEBUG("Call resize(mask)");
    cv::Mat binaryMask;
    cv::compare(inputMask, 0, binaryMask, cv::CMP_GT);
    cv::resize(binaryMask, currentMask, targetSize, 0, 0, cv::INTER_NEAREST);
  }

  if ( _applyDifferentiate ) {
    CF_DEBUG("Call compute_gradient(currentImage)");
    compute_gradient(currentImage, currentGradientImage, currentMask);
  }
  else {
    currentGradientImage = currentImage;
  }

  if ( !_applyMaskApodization || currentMask.empty() ) {
    maskedCurrentImage = currentGradientImage;
  }
  else {
    CF_DEBUG("Call Apodization(currentImage)");
    createApodizationWindowFromMask(currentMask, window, scaledRadius);
    cv::multiply(currentGradientImage, window, maskedCurrentImage);
  }

  if ( _display == DISPLAY_CURRENT_IMAGE ) {
    maskedCurrentImage.copyTo(image);
    if ( !currentMask.empty() ) {
      currentMask.copyTo(mask);
    }
    else {
      mask.release();
    }
    goto end;
  }

  if ( prevImage.empty() ) {
    if ( _updatePrevious ) {
      CF_DEBUG("Call Update prevImage");
      currentImage.copyTo(prevImage);
      if (!currentMask.empty()) {
        currentMask.copyTo(prevMask);
      }
    }
    return true;
  }

  if( prevImage.size() != currentImage.size() ) {
    CF_ERROR("Scaled current (%dx%d) and previous (%dx%d) sizes do not match. \n"
        "Use 'updatePrevious' checkbox to reset new reference",
        currentImage.cols, currentImage.rows,
        prevImage.cols, prevImage.rows);
    goto end;
  }

  if ( _applyDifferentiate ) {
    CF_DEBUG("Call compute_gradient(prevImage)");
    compute_gradient(prevImage, prevGradientImage, prevMask);
  }
  else {
    prevGradientImage = prevImage;
  }

  if ( !_applyMaskApodization || prevMask.empty() ) {
    maskedPreviousImage = prevGradientImage;
  }
  else {
    CF_DEBUG("Call Apodization(prevImage)");
    createApodizationWindowFromMask(prevMask, window, scaledRadius);
    cv::multiply(prevGradientImage, window, maskedPreviousImage);
  }

  if ( _display == DISPLAY_PREVIOUS_IMAGE ) {
    maskedPreviousImage.copyTo(image);
    if (!prevMask.empty()) {
      prevMask.copyTo(mask);
    }
    else {
      mask.release();
    }
    goto end;
  }

  CF_DEBUG("Call dct(CurrentImage)");
  cv::dct(maskedCurrentImage, dctCurrent);
  if ( _display == DISPLAY_DCT_CURRENT ) {
    image.move(dctCurrent);
    mask.release();
    goto end;
  }

  CF_DEBUG("Call dct(PreviousImage)");
  cv::dct(maskedPreviousImage, dctPrevious);
  if ( _display == DISPLAY_DCT_PREVIOUS ) {
    image.move(dctPrevious);
    mask.release();
    goto end;
  }

  CF_DEBUG("Call dctCross");
  cv::multiply(dctCurrent, dctPrevious, dctCross);

  if ( _applyRAMP ) {
    CF_DEBUG("Call applyRAMP");
    if ( RAMP.size() != dctCross.size()) {
      RAMP = dctGenerateRampFilter(dctCross.size(), 1);
    }
    cv::multiply(dctCross, RAMP, dctCross);
  }

  if ( _display == DISPLAY_DCT_CROSS ) {
    image.move(dctCross);
    mask.release();
    goto end;
  }

  CF_DEBUG("Call idct(dctCross)");
  cv::idct(dctCross, idctCross);

  CF_DEBUG("Call findSubpixelShiftDCT()");
  peakLocation = findSubpixelShiftDCT(idctCross);

  {
    image.move(idctCross);
    mask.release();
    goto end;
  }


end:
  CF_DEBUG("Finsh. peakLocation: x=%g y=%g", peakLocation.x, peakLocation.y);
  if ( _updatePrevious ) {
    currentImage.copyTo(prevImage);
    currentMask.copyTo(prevMask);
  }

  return true;
}
