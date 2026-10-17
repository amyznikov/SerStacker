/*
 * c_fft_polar_phase_correlate_routine.cc
 *
 *  Created on: Oct 16, 2026
 *      Author: amyznikov
 *
 *
 * «An FFT-based technique for translation, rotation, and scale-invariant image registration»
 *   (B. S. Reddy, B. N. Chatterji, 1996).
 *
 * «Extension of Phase Correlation to Rotation, Scale and Translation Invariant Image Registration»
 * (Emmanuel Sarvaiya, S. Patnaik).
 *
 * «Image Registration Using Log Polar Transform and Phase Correlation to Recover Higher Scale»
 * (Journal of Pattern Recognition Research, 2012).
 */

#include "c_fft_polar_phase_correlate_routine.h"
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_fft_polar_phase_correlate_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_IMAGE, "REFERENCE_IMAGE" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_SPECTRUM, "CURRENT_SPECTRUM", },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_SPECTRUM, "REFERENCE_SPECTRUM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM, "CURRENT_ACM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_ACM, "REFERENCE_ACM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM_POLAR, "CURRENT_ACM_POLAR" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_ACM_POLAR, "REFERENCE_ACM_POLAR" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM_POLAR_COUNTER, "CURRENT_ACM_POLAR_COUNTER" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_ACM_POLAR_COUNTER, "REFERENCE_ACM_POLAR_COUNTER" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM, },
  };
  return members;
}

extern cv::Mat1f fftCreateEllipticalApodizationWindow(const cv::Size & size, double edgeSize);

//static cv::Size computePackedImageSize(const cv::Size imageSize, const cv::Size & fftSize)
//{
//  const int downW = (imageSize.width + 1) / 2;
//  const int downH = (imageSize.height + 1) / 2;
//  const cv::Size smallSize(downW, downH);
//  const int cropW = std::min(smallSize.width, fftSize.width);
//  const int cropH = std::min(smallSize.height, fftSize.height);
//
//}

static void packScaledImageForPhaseCorreation(cv::InputArray srcImage, cv::InputArray srcMask,
    cv::Mat1f & outImage, cv::Mat1b & outMask,
    const cv::Size & fftSize,
    cv::Size & outputValidSize,
    cv::Point & cropOffset,
    bool maxPooling)
{
  cv::Mat srcMat = srcImage.getMat();

  const int downW = (srcMat.cols + 1) / 2;
  const int downH = (srcMat.rows + 1) / 2;
  const cv::Size smallSize(downW, downH);

  cv::Mat smallImage, smallMask;

  if (!maxPooling) {
    cv::pyrDown(srcMat, smallImage, smallSize);
  }
  else {
    cv::Mat dilated;
    static const cv::Mat1b SE(3, 3, 255);
    cv::dilate(srcMat, dilated, SE);
    cv::pyrDown(dilated, smallImage, smallSize);
  }

  if (smallImage.depth() != CV_32F) {
    smallImage.convertTo(smallImage, CV_32F);
  }

  if (!srcMask.empty()) {
    cv::resize(srcMask, smallMask, smallSize, 0, 0, cv::INTER_NEAREST);
  }

  cv::Mat croppedImage, croppedMask;
  if (smallImage.cols <= fftSize.width && smallImage.rows <= fftSize.height) {
    cropOffset = cv::Point(0, 0);
    croppedImage = smallImage;
    croppedMask = smallMask;
    outputValidSize = smallImage.size();
  }
  else {
    const int cropW = std::min(smallImage.cols, fftSize.width);
    const int cropH = std::min(smallImage.rows, fftSize.height);
    cropOffset.x = (smallImage.cols - cropW) / 2;
    cropOffset.y = (smallImage.rows - cropH) / 2;
    const cv::Rect roi(cropOffset.x, cropOffset.y, cropW, cropH);
    croppedImage = smallImage(roi);
    if (!smallMask.empty()) {
      croppedMask = smallMask(roi);
    }
    outputValidSize = cv::Size(cropW, cropH);
  }

  if ( true ) {
    cv::subtract(croppedImage, cv::mean(croppedImage), croppedImage);
    cv::Mat1f w;
    generateTukeyApodizationWindow(w, croppedImage.size(), 0.2);
    //const cv::Mat1f w = fftCreateEllipticalApodizationWindow(croppedImage.size(), 100);
    cv::multiply(croppedImage, w, croppedImage);

  }

  outImage.create(fftSize);
  outImage.setTo(0);
  outMask.create(fftSize);
  outMask.setTo(0);

  const int targetX = (fftSize.width - outputValidSize.width) / 2;
  const int targetY = (fftSize.height - outputValidSize.height) / 2;
  const cv::Rect centerROI(targetX, targetY, outputValidSize.width, outputValidSize.height);
  croppedImage.copyTo(outImage(centerROI));

  if (croppedMask.empty()) {
    outMask(centerROI).setTo(255);
  }
  else {
    croppedMask.copyTo(outMask(centerROI));
  }
}

static int getOptimalFFTSizeDown(int size)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  int sopt = cv::getOptimalDFTSize(size);
  while ( sopt > 0 && ((sopt > size) || (sopt & 0x1)) ) {
    sopt = cv::getOptimalDFTSize(--size);
  }
  return sopt;
}

static void acm2plr(const cv::Mat1f & acm, cv::Mat1f & acmplr, cv::Mat1f & cntr, int nAngles)
{
  const cv::Size acmSize = acm.size();
  const float cx = acmSize.width / 2.f;
  const float cy = acmSize.height / 2.f;
  const int rmin = (int) (nAngles / (CV_PI));
  const int rmax = std::min(acmSize.width, acmSize.height) / 2;
  const int base_radii = rmax - rmin;
  const int nRadii = getOptimalFFTSizeDown(base_radii);

  const float rscale = float(nRadii - 1) / (rmax - rmin);
  const float tscale = float((nAngles) / (CV_PI));

  const cv::Size lpSize(nRadii, nAngles);

  CF_DEBUG("acmSize: %dx%d lpSize: %dx%d rmin=%d rmax=%d base_radii=%d nRadii=%d rscale=%g tscale=%g",
      acmSize.width, acmSize.height,
      lpSize.width, lpSize.height,
      rmin, rmax, base_radii, nRadii,
      rscale, tscale);

  const cv::Size paddedSize(nRadii, nAngles + 2);

  cv::Mat1f padded_cntr(paddedSize, 0.f);
  cv::Mat1f padded_acmplr(paddedSize, 0.f);

  const uint8_t * const src_base = acm.ptr();
  const size_t src_stride = acm.step;

  uint8_t * const acmplr_base = padded_acmplr.ptr();
  const size_t acmplr_stride = padded_acmplr.step;

  uint8_t * const cntr_base = padded_cntr.ptr();
  const size_t cntr_stride = padded_cntr.step;

  for( int y = 0; y < acmSize.height / 2; ++y ) {

    const float * srcp = (const float * )(src_base + y * src_stride);

    const float dy = y - cy;
    const float dy2 = dy * dy;

    for( int x = 0; x < acmSize.width; ++x ) {

      const float dx = x - cx;
      const float dx2 = dx * dx;

      const float rf = (std::sqrt(dx2 + dy2) - rmin) * rscale;
      if ( rf < 0.f || rf >= float(nRadii - 1) ) {
        continue;
      }

      const float angle = std::atan2(-dy, dx);
      const float tf = angle * tscale + 1.f;

      const int r0 = int(rf);
      const int r1 = r0 + 1;
      const int t0 = int(tf);
      const int t1 = t0 + 1;

      const float dr = rf - float(r0);
      const float dt = tf - float(t0);

      const float w00 = (1.f - dt) * (1.f - dr);
      const float w10 = dt * (1.f - dr);
      const float w01 = (1.f - dt) * dr;
      const float w11 = dt * dr;

      const float val = std::abs(srcp[x]);

      float * __restrict acmplr_t0 = (float*) (acmplr_base + t0 * acmplr_stride);
      float * __restrict acmplr_t1 = (float*) (acmplr_base + t1 * acmplr_stride);

      float * __restrict cntr_t0   = (float*) (cntr_base + t0 * cntr_stride);
      float * __restrict cntr_t1   = (float*) (cntr_base + t1 * cntr_stride);

      acmplr_t0[r0] += val * w00;   cntr_t0[r0] += w00;
      acmplr_t1[r0] += val * w10;   cntr_t1[r0] += w10;
      acmplr_t0[r1] += val * w01;   cntr_t0[r1] += w01;
      acmplr_t1[r1] += val * w11;   cntr_t1[r1] += w11;
    }
  }

  divideImages(padded_acmplr, padded_cntr, padded_acmplr);
  const cv::Rect roi(0, 1, nRadii, nAngles);
  acmplr = padded_acmplr(roi);
  cntr   = padded_cntr(roi);
}

cv::Size c_fft_polar_phase_correlate_routine::computeFFTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  const int W = cvRound(expectedFrameSize.width / downscaleFactor);
  const int H = cvRound(expectedFrameSize.height / downscaleFactor);
  const int downscaledW = getOptimalFFTSizeDown(W);
  const int downscaledH = getOptimalFFTSizeDown(H);
  return cv::Size(std::max(4, downscaledW), std::max(4, downscaledH));
}

void c_fft_polar_phase_correlate_routine::generateBandpassFilter1(cv::Mat1f & outputFilter,
    const cv::Size & fftSize,
    double gsigma)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  // ilambda = gsigma * (2 * sqrt(pi))
  // rho2 = (u^2 + v^2) * ilambda^2;
  // F(u, v) = rho2 * exp (-rho2/2 )
  // the alternating +1 and -1 is also embedded into filter to avoid later fftSwapQudrants()

  INSTRUMENT_REGION("");

  outputFilter.create(fftSize);

  const uint8_t * filter_base = outputFilter.ptr();
  const size_t filter_stride = outputFilter.step;

  const int cols = fftSize.width;
  const int rows = fftSize.height;

  if ( gsigma <= 0 ) {
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
    const double ilambda_x = (2.0 * CV_PI * gsigma) / cols;
    const double ilambda_y = (2.0 * CV_PI * gsigma) / rows;
    const float ilambda_x2 = float(ilambda_x * ilambda_x);
    const float ilambda_y2 = float(ilambda_y * ilambda_y);

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);

        const int fy = (y > rows / 2) ? (rows - y) : y;
        const float v2 = float(fy * fy * ilambda_y2);

        for( int x = 0; x < cols; ++x ) {
          const int fx = (x > cols / 2) ? (cols - x) : x;
          const int sign = ((x + y) & 1) ? -1 : 1;
          const float u2 = float(fx * fx * ilambda_x2);
          const float rho2 = u2 + v2;
          const float fltv = rho2 * std::exp(-0.5f * rho2);
          fltp[x] = sign * fltv;
        }
      }
    });
  }

  cv::multiply(outputFilter, 1. / cv::norm(outputFilter, cv::NORM_L1),
      outputFilter);
}

void c_fft_polar_phase_correlate_routine::generateBandpassFilter2(cv::Mat1f & outputFilter,
    const cv::Size & fftSize,
    double gsigma)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  // ilambda = gsigma * (2 * sqrt(pi))
  // rho2 = (u^2 + v^2) * ilambda^2;
  // F(u, v) = rho2 * rho2 * exp (-rho2 )
  // the alternating +1 and -1 is also embedded into filter to avoid later fftSwapQudrants()

  INSTRUMENT_REGION("");

  outputFilter.create(fftSize);

  const uint8_t * filter_base = outputFilter.ptr();
  const size_t filter_stride = outputFilter.step;

  const int cols = fftSize.width;
  const int rows = fftSize.height;

  if ( gsigma <= 0 ) {
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
    const double ilambda_x = (2.0 * CV_PI * gsigma) / cols;
    const double ilambda_y = (2.0 * CV_PI * gsigma) / rows;
    const float ilambda_x2 = float(ilambda_x * ilambda_x);
    const float ilambda_y2 = float(ilambda_y * ilambda_y);

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);

        const int fy = (y > rows / 2) ? (rows - y) : y;
        const float v2 = float(fy * fy * ilambda_y2);

        for( int x = 0; x < cols; ++x ) {
          const int fx = (x > cols / 2) ? (cols - x) : x;
          const int sign = ((x + y) & 1) ? -1 : 1;
          const float u2 = float(fx * fx * ilambda_x2);
          const float rho2 = u2 + v2;
          const float fltv = rho2 * rho2 * std::exp(-rho2);
          fltp[x] = sign * fltv;
        }
      }
    });
  }

  cv::multiply(outputFilter, 1. / cv::norm(outputFilter, cv::NORM_L1),
      outputFilter);
}

bool c_fft_polar_phase_correlate_routine::setupInputImage(cv::InputArray srcImage, cv::InputArray srcMask,
    const cv::Mat1f & VLAP,
    cv::Mat1f & outScaledImage, cv::Mat1b & outScaledMask,
    cv::Size & outValidSize, cv::Point & outCropOffset,
    cv::Mat1f & outCCSSpectrum) const
{
  INSTRUMENT_REGION("");

  const cv::Size fftSize = VLAP.size();
  if( fftSize.empty() ) {
    CF_ERROR("fftSize is empty");
    return false;
  }

  if( srcImage.empty() ) {
    CF_ERROR("input image is empty");
    return false;
  }

  if( srcImage.channels() != 1 || (!srcMask.empty() && srcMask.channels() != 1) ) {
    CF_ERROR("c_phase_correlate: input image and mask must be single-channel");
    return false;
  }

  packScaledImageForPhaseCorreation(srcImage, srcMask,
      outScaledImage, outScaledMask, fftSize,
      outValidSize,
      outCropOffset,
      _maxPooling);

  // Call DFT with Virginie Moizan Periodic + Smooth decomposition in OpenCV CCS format
  if( !fftPPSDecompositionCCS(outScaledImage, VLAP, outCCSSpectrum, cv::noArray()) ) {
    CF_ERROR("fftPPSDecompositionCCS() fails");
    return false;
  }

  return true;
}

bool  c_fft_polar_phase_correlate_routine::setupInputFrame(cv::InputArray srcImage, cv::InputArray srcMask,
    Frame * frame) const
{
  cv::Mat src;
  if ( srcImage.channels() == 1 ) {
    if ( srcImage.depth() == CV_32F ) {
      src = srcImage.getMat();
    }
    else {
      srcImage.getMat().convertTo(src, CV_32F);
    }
  }
  else if ( srcImage.depth() == CV_32F ) {
    cv::cvtColor(srcImage, src, cv::COLOR_BGR2GRAY);
  }
  else {
    cv::cvtColor(srcImage, src, cv::COLOR_BGR2GRAY);
    src.convertTo(src, CV_32F);
  }

  bool fOK =
      setupInputImage(src, srcMask, vlap,
          frame->image, frame->mask,
          frame->validSize,
          frame->cropOffset,
          frame->spec);

  if ( !fOK ) {
    CF_ERROR("setupInputImage() fails");
    return false;
  }

  if ( fftAutoCrossSpectrumWeightedCCS(frame->spec, bp, frame->spec) <= 0 ) {
    CF_ERROR("fftAutoCrossSpectrumWeightedCCS() fails");
    return false;
  }

  cv::idft(frame->spec, frame->acm, cv::DFT_REAL_OUTPUT);

  acm2plr(frame->acm, frame->acmPolar, frame->acmPolarMask, _numAngles);

  return true;
}


bool c_fft_polar_phase_correlate_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _numAngles);
    SERIALIZE_OPTION(settings, save, *this, _gsimga);
    SERIALIZE_OPTION(settings, save, *this, _maxPooling);
    return true;
  }
  return false;
}

void c_fft_polar_phase_correlate_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display), "display");
  ctlbind(ctls, "numAngles", ctx(&this_class::_numAngles), "numAngles");
  ctlbind(ctls, "gsimga", ctx(&this_class::_gsimga), "gsimga");
  ctlbind(ctls, "maxPooling", ctx(&this_class::_maxPooling), "");
  ctlbind(ctls, "updateReference", ctx(&this_class::_updateReference), "updateReference");
}

bool c_fft_polar_phase_correlate_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _updateReference || referenceFrame.image.empty() ) {

    const cv::Size fftSize = computeFFTPackSize(image.size(), 1);
    if( vlap.size() != fftSize ) {
      vlap = fftGenerateDiscreteLaplacianFilter(fftSize, false);
    }

    generateBandpassFilter1(bp, fftSize, _gsimga);

    if ( !setupInputFrame(image, mask, &referenceFrame) ) {
      CF_ERROR("setupInputFrame(referenceFrame) fails");
      return false;
    }
  }


  bool fOK = setupInputFrame(image, mask, &currentFrame);
  if ( !fOK ) {
    CF_ERROR("setupInputFrame(currentFrame) fails");
    return false;
  }


  switch(_display) {
    case DISPLAY_CURRENT_IMAGE:
      currentFrame.image.copyTo(image);
      mask.release();
      break;
    case DISPLAY_REFERENCE_IMAGE:
      referenceFrame.image.copyTo(image);
      mask.release();
      break;
    case DISPLAY_CURRENT_SPECTRUM:
      fftCCSSpectrumMagnitude(currentFrame.spec, image, true);
      mask.release();
      break;
    case DISPLAY_REFERENCE_SPECTRUM:
      fftCCSSpectrumMagnitude(referenceFrame.spec, image, true);
      mask.release();
      break;
    case DISPLAY_CURRENT_ACM:
      currentFrame.acm.copyTo(image);
      mask.release();
      break;
    case DISPLAY_REFERENCE_ACM:
      referenceFrame.acm.copyTo(image);
      mask.release();
      break;
    case DISPLAY_CURRENT_ACM_POLAR:
      currentFrame.acmPolar.copyTo(image);
      cv::compare(currentFrame.acmPolarMask, 0, mask, cv::CMP_GT);
      break;
    case DISPLAY_REFERENCE_ACM_POLAR:
      referenceFrame.acmPolar.copyTo(image);
      cv::compare(referenceFrame.acmPolarMask, 0, mask, cv::CMP_GT);
      break;
    case DISPLAY_CURRENT_ACM_POLAR_COUNTER:
      currentFrame.acmPolarMask.copyTo(image);
      mask.release();
      break;
    case DISPLAY_REFERENCE_ACM_POLAR_COUNTER:
      referenceFrame.acmPolarMask.copyTo(image);
      mask.release();
      break;

  }


  return true;
}

