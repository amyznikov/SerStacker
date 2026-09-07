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
      { c_alpha_test_routine::DISPLAY_FFT_CROSS, "FFT_CROSS", "" },
      { c_alpha_test_routine::DISPLAY_IFFT, "IFFT", "" },
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

void c_phase_correlate::differentiate(cv::InputArray src, cv::OutputArray dst, cv::InputArray mask)
{
  // TODO: Later consider something like laplacian or DoG kernel instead,
  // as it is expected to be faster an better preserving the sign of edges

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

  cv::magnitude(gx, gy, dst);

  if ( !mask.empty() ) {
    dst.setTo(0, ~mask.getMat());
  }
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

  // Финальное сглаживание бокс-фильтром
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
//  if( subX >= cols / 2.0 ) {
//    subX -= cols;
//  }
//  if( subY >= rows / 2.0 ) {
//    subY -= rows;
//  }

  return cv::Point2f(float(subX), float(subY));
}

// Modifying the RAMP generator to freely move the zero offset to the center of idftResult
void c_phase_correlate::generateRampFilter(const cv::Size & size, cv::Mat2f & outputFilter ) const
{
  cv::Mat1f FILTER(size);

  const float scaleX = float(CV_2PI / size.width);
  const float scaleY = float(CV_2PI / size.height);
  const int cx = size.width / 2;
  const int cy = size.height / 2;

  parallel_for(0, size.height, [=, &FILTER](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = FILTER.ptr<float>(y);

      const float dy_val = (y <= cy) ? float(y) : float(size.height - y);
      const float dy = dy_val * scaleY;
      const float dy2 = dy * dy;

      for (int x = 0; x < size.width; ++x) {
        const float dx_val = (x <= cx) ? float(x) : float(size.width - x);
        const float dx = dx_val * scaleX;

        float radius = std::sqrt(dx * dx + dy2);

        // If the sum of the frequency indices (x + y) is odd, make the RAMP coefficient negative.
        // When multiplying the spectrum, this inverts the phase of the frequencies, which is mathematically
        // equivalent to an honest spatial fftShift() after the IDFT!
        if ((x + y) % 2 != 0) {
          radius = -radius;
        }

        dstp[x] = radius;
      }
    }
  });

  FILTER(0, 0) = 0.0f;

  const cv::Mat m[2] = {
      FILTER, FILTER
  };

  cv::merge(m, 2, outputFilter);
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

  cv::Size targetSize = getOptimalPhaseCorrelationSize(inputCurrent.size());

  if (targetSize != _cachedSize) {
    _cachedSize = targetSize;
    _cachedScaleFactor = double(targetSize.width) / inputCurrent.cols;
  }

  if( _opts.apply_ramp && _rampFilter.size() != _cachedSize ) {
    generateRampFilter(_cachedSize, _rampFilter);
  }

  const int scaledRadius = std::max(1, cvRound(_opts.apodization_radius * _cachedScaleFactor));

  scaleAndPadToTargetSize(currentImage, currentMask, _scaledImg1, _scaledMsk1, _cachedSize, _cachedScaleFactor);
  scaleAndPadToTargetSize(referenceImage, referenceMask, _scaledImg2, _scaledMsk2, _cachedSize, _cachedScaleFactor);

  if (_opts.differentiate) {
    differentiate(_scaledImg1, _gradImg1, _scaledMsk1);
    differentiate(_scaledImg2, _gradImg2, _scaledMsk2);
  }
  else {
    _scaledImg1.copyTo(_gradImg1);
    _scaledImg2.copyTo(_gradImg2);
  }

  if( !_opts.apply_apodization ) {
    _gradImg1.copyTo(_maskedImg1);
    _gradImg2.copyTo(_maskedImg2);
  }
  else {
    createApodizationWindow(_scaledMsk1, _window1, scaledRadius);
    createApodizationWindow(_scaledMsk2, _window2, scaledRadius);
    cv::multiply(_gradImg1, _window1, _maskedImg1);
    cv::multiply(_gradImg2, _window2, _maskedImg2);
  }

  cv::dft(_maskedImg1, _fft1, cv::DFT_COMPLEX_OUTPUT);
  cv::dft(_maskedImg2, _fft2, cv::DFT_COMPLEX_OUTPUT);

  cv::mulSpectrums(_fft1, _fft2, _fftCross, 0, true);

  if ( _opts.apply_ramp ) {
    cv::multiply(_fftCross, _rampFilter, _fftCross);
  }
  cv::idft(_fftCross, realInverseResult, cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
  if ( !_opts.apply_ramp ) {
    fftSwapQuadrants(realInverseResult);
  }

  cv::Point peakLoc;
  const cv::Point2f scaledTranslation = findSubpixelCentroid(realInverseResult, peakLoc);
  const double totalSpectrumEnergy = cv::norm(_fftCross, cv::NORM_L1);
  const double normalizedEnergy = totalSpectrumEnergy / (realInverseResult.cols * realInverseResult.rows);
  const double rawPeak = realInverseResult(peakLoc.y, peakLoc.x);
  const double correlationScore = (normalizedEnergy > 0.0) ? (rawPeak / normalizedEnergy) : 0.0;

  if( outputTranslation ) {
    (*outputTranslation)[0] = (scaledTranslation.x - realInverseResult.cols / 2) * float(1.0 / _cachedScaleFactor);
    (*outputTranslation)[1] = (scaledTranslation.y - realInverseResult.rows / 2) * float(1.0 / _cachedScaleFactor);
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
    SERIALIZE_OPTION(settings, save, pc._opts, differentiate);
    SERIALIZE_OPTION(settings, save, pc._opts, apply_apodization);
    SERIALIZE_OPTION(settings, save, pc._opts, apply_ramp);
    SERIALIZE_OPTION(settings, save, pc._opts, apodization_radius);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "Differentiate", CTL_CONTEXT(ctx, pc._opts.differentiate), "");
  ctlbind(ctls, "applyRAMP", CTL_CONTEXT(ctx, pc._opts.apply_ramp), "Set checked to apply RAMP filter to dct cross");
  ctlbind(ctls, "applyApodization", CTL_CONTEXT(ctx, pc._opts.apply_apodization), "");
  ctlbind(ctls, "ApodizationRadius", CTL_CONTEXT(ctx, pc._opts.apodization_radius), "");
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
      case DISPLAY_FFT_CROSS:
        cv::extractChannel(pc._fftCross, image, 0);
        mask.release();
        break;
      case DISPLAY_IFFT:
        pc.realInverseResult.copyTo(image);
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

