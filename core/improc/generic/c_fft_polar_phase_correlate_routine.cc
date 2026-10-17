/*
 * c_fft_polar_phase_correlate_routine.cc
 *
 *  Created on: Oct 16, 2026
 *      Author: amyznikov
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
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_SPECTRUM, "CURRENT_SPECTRUM", },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_SPECTRUM, "REFERENCE_SPECTRUM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CUREENT_SPECTRUM_LP, "CUREENT_SPECTRUM_LP" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_SPECTRUM_LP, "REFERENCE_SPECTRUM_LP" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CORRELATION_MAP, "DISPLAY_CORRELATION_MAP" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CORRELATION_MAP, },
  };
  return members;
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

static void fftSpec2LogPolar(const cv::Mat1f & cartMag, cv::Mat1f & logPolarMag, int nUniform, int nAngles)
{
  const cv::Size fftSize = cartMag.size();
  const float centerX = fftSize.width / 2;
  const float centerY = fftSize.height / 2;
  const float tscale = float((nAngles - 1) / (2.0 * CV_PI));

  const int minFreq = (int) (nAngles / (2.0 * CV_PI));
  const int maxFreq = std::min(fftSize.width, fftSize.height) / 2;
  const int nBins = maxFreq - minFreq + 1;
  const float logMinFreq = std::log(minFreq);
  const float logMaxFreq = std::log(maxFreq);
  const float logFreqScale = (nUniform - 1) / (logMaxFreq - logMinFreq);

  std::vector<int> uniformIndexes(nBins);
  for( int i = 0; i < nBins; ++i ) {
    const float f = (i + minFreq);
    uniformIndexes[i] = std::clamp(int((std::log(f) - logMinFreq) * logFreqScale), 0, nBins - 1);
  }

  logPolarMag.create(nAngles, nUniform);
  logPolarMag.setTo(0);

  cv::Mat1f logPolarCounts(nAngles, nUniform, 0.f);

  const uint8_t * const src_base = cartMag.ptr();
  const size_t src_stride = cartMag.step;

  uint8_t * const outmap_base = logPolarMag.ptr();
  const size_t outmap_stride = logPolarMag.step;

  uint8_t * const counts_base = logPolarCounts.ptr();
  const size_t counts_stride = logPolarCounts.step;

  for( int y = 0; y < fftSize.height; ++y ) {
    const float dy = y - centerY;
    const float dy2 = dy * dy;

    const float * srcp = (const float*) (src_base + y * src_stride);

    for( int x = 0; x < fftSize.width; ++x ) {
      const float dx = x - centerX;
      const float dx2 = dx * dx;
      const float dr = std::sqrt(dx2 + dy2);
      const int r = int(dr) - minFreq;
      if( r < 0 || r >= nBins ) {
        continue;
      }

      const int t = int((std::atan2(dy, dx) + CV_PI) * tscale);
      float * __restrict outmap = (float*) (outmap_base + t * outmap_stride);
      float * __restrict counts = (float*) (counts_base + t * counts_stride);

      const int rbin = uniformIndexes[r];
      outmap[rbin] += srcp[x];
      counts[rbin] += 1;
    }
  }

  divideImages(logPolarMag, logPolarCounts, logPolarMag);

  cv::Mat avg, holes;
  cv::compare(logPolarCounts, 0.9, holes, cv::CMP_LT);
  cv::medianBlur(logPolarMag, avg, 5);
  //cv::morphologyEx(logPolarMag, avg, cv::MORPH_CLOSE, cv::Mat1b(5, 5, 255), cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
  avg.copyTo(logPolarMag, holes);
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
  INSTRUMENT_REGION("");

  outputFilter.create(fftSize);

  const uint8_t * filter_base = outputFilter.ptr();
  const size_t filter_stride = outputFilter.step;

  const int cols = fftSize.width;
  const int rows = fftSize.height;

  if ( gsigma <= 0 ) {
    outputFilter.setTo(1);
  }
  else {
    const double ilambda_x = (2.0 * CV_PI * gsigma) / cols;
    const double ilambda_y = (2.0 * CV_PI * gsigma) / rows;
    const float ilambda_x2 = float(ilambda_x * ilambda_x);
    const float ilambda_y2 = float(ilambda_y * ilambda_y);
    const float cx = cols / 2;
    const float cy = rows / 2;

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);

        const int fy = y - cy;
        const float v2 = float(fy * fy * ilambda_y2);

        for( int x = 0; x < cols; ++x ) {
          const int fx = x - cx;
          const float u2 = float(fx * fx * ilambda_x2);
          const float rho2 = u2 + v2;
          const float fltv = rho2 * std::exp(-0.5f * rho2);
          fltp[x] = fltv;
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

  cv::Mat smallImage, smallMask;
  if( std::abs(_downscale_factor - 1) < 10 * FLT_EPSILON ) {
    smallImage = srcImage.getMat();
    smallMask = srcMask.getMat();
  }
  else {
    const double scale = 1.0 / _downscale_factor;
    cv::resize(srcImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
    if( !srcMask.empty() ) {
      cv::resize(srcMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
    }
  }

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      outScaledImage, outScaledMask, fftSize,
      outValidSize,
      outCropOffset);

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
      setupInputImage(src, srcMask, vlap1,
          frame->image, frame->mask,
          frame->validSize,
          frame->cropOffset,
          frame->imageSpec);

  if ( !fOK ) {
    CF_ERROR("setupInputImage() fails");
    return false;
  }

  if ( !fftCCSSpectrumMagnitude(frame->imageSpec, frame->imageSpec, true) ) {
    CF_ERROR("fftCCSSpectrumMagnitude(imageSpec) fails");
    return false;
  }

  // apply bandpass filter
  cv::multiply(frame->imageSpec, bp1, frame->imageSpec);

  fftSpec2LogPolar(frame->imageSpec, frame->imageSpecLogPolar, numRadii, _numAngles);
  if( !fftPPSDecompositionCCS(frame->imageSpecLogPolar, vlap2, frame->imageSpecLogPolarSpec, cv::noArray()) ) {
    CF_ERROR("fftPPSDecompositionCCS(imageSpecLogPolar) fails");
    return false;
  }

  return true;
}


bool c_fft_polar_phase_correlate_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _numAngles);
    SERIALIZE_OPTION(settings, save, *this, _gsimga1);
    SERIALIZE_OPTION(settings, save, *this, _gsimga2);
    return true;
  }
  return false;
}

void c_fft_polar_phase_correlate_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display), "display");
  ctlbind(ctls, "numAngles", ctx(&this_class::_numAngles), "numAngles");
  ctlbind(ctls, "gsimga1", ctx(&this_class::_gsimga1), "gsimga1");
  ctlbind(ctls, "gsimga2", ctx(&this_class::_gsimga2), "gsimga2");
  ctlbind(ctls, "updateReference", ctx(&this_class::_updateReference), "updateReference");
}

bool c_fft_polar_phase_correlate_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _updateReference || referenceFrame.image.empty() ) {

    const cv::Size fft1Size = computeFFTPackSize(image.size(), _downscale_factor);
    if( vlap1.size() != fft1Size ) {
      vlap1 = fftGenerateDiscreteLaplacianFilter(fft1Size, false);
    }

    const cv::Size fft2Size (numRadii, _numAngles);
    if( vlap2.size() != fft2Size ) {
      vlap2 = fftGenerateDiscreteLaplacianFilter(fft2Size, false);
    }

    generateBandpassFilter1(bp1, fft1Size, _gsimga1);
    generateBandpassFilter2(bp2, fft2Size, _gsimga2);

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

  cv::Mat1f crossSpectrum, correlationMap;
#if 0
  double crossEnergy =
      fftCrossSpectrumWeightedCCS(currentFrame.imageSpecLogPolarSpec,
          referenceFrame.imageSpecLogPolarSpec, bp2, crossSpectrum);
  if( crossEnergy <= 0 ) {
    CF_ERROR("fftCrossSpectrumWeightedCCS() fails");
    return false;
  }
#else
  fOK =
      fftPhaseCorrelateWeightedCCS(currentFrame.imageSpecLogPolarSpec,
          referenceFrame.imageSpecLogPolarSpec, bp2, crossSpectrum);
  if( !fOK ) {
    CF_ERROR("fftPhaseCorrelateWeightedCCS() fails");
    return false;
  }
#endif

  cv::idft(crossSpectrum, correlationMap,
      cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);


  switch(_display) {
    case DISPLAY_CURRENT_SPECTRUM:
      currentFrame.imageSpec.copyTo(image);
      break;
    case DISPLAY_REFERENCE_SPECTRUM:
      referenceFrame.imageSpec.copyTo(image);
      break;
    case DISPLAY_CUREENT_SPECTRUM_LP:
      currentFrame.imageSpecLogPolar.copyTo(image);
      break;
    case DISPLAY_REFERENCE_SPECTRUM_LP:
      referenceFrame.imageSpecLogPolar.copyTo(image);
      break;
    case DISPLAY_CORRELATION_MAP:
      correlationMap.copyTo(image);
      break;
  }

  mask.release();

  return true;
}

