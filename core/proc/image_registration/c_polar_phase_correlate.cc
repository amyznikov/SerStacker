/*
 * c_fft_polar_phase_correlate.cc
 *
 *  Created on: Oct 17, 2026
 *      Author: amyznikov
 */

#include "c_polar_phase_correlate.h"
#include <core/proc/run-loop.h>
#include <core/proc/fft.h>
#include <core/proc/divide.h>
#include <core/ssprintf.h>
#include <core/debug.h>

static int getOptimalFFTSizeDown(int size)
{
  int sopt = cv::getOptimalDFTSize(size);
  while ( sopt > 0 && ((sopt > size) || (sopt & 0x1)) ) {
    sopt = cv::getOptimalDFTSize(--size);
  }
  return sopt;
}

static void acm2plr(const cv::Mat1f & acm, cv::Mat1f & acmplr, int nAngles)
{
  const cv::Size acmSize = acm.size();
  const float cx = acmSize.width / 2;
  const float cy = acmSize.height / 2;
  const int rmin = (int) (nAngles / (CV_PI));
  const int rmax = std::min(acmSize.width, acmSize.height) / 2;
  const int base_radii = rmax - rmin;
  const int nRadii = getOptimalFFTSizeDown(base_radii);

  const float rscale = float(nRadii - 1) / (rmax - rmin);
  const float tscale = float((nAngles) / (CV_PI));

  const cv::Size lpSize(nRadii, nAngles);

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
  acmplr = padded_acmplr(cv::Rect(0, 1, nRadii, nAngles));
}

static void packScaledImageForPhaseCorreation(cv::InputArray srcImage, cv::Mat1f & outImage,
    const cv::Size & fftSize,
    cv::Size & outputValidSize,
    cv::Point & cropOffset)
{
  cv::Mat smallImage;

  if (srcImage.cols() <= fftSize.width && srcImage.rows() <= fftSize.height) {
    cropOffset = cv::Point(0, 0);
    smallImage = srcImage.getMat();
    outputValidSize = smallImage.size();
  }
  else {
    const int cropW = std::min(srcImage.cols(), fftSize.width);
    const int cropH = std::min(srcImage.rows(), fftSize.height);
    cropOffset.x = (srcImage.cols() - cropW) / 2;
    cropOffset.y = (srcImage.rows() - cropH) / 2;
    const cv::Rect roi(cropOffset.x, cropOffset.y, cropW, cropH);
    smallImage = srcImage.getMat()(roi);
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
  }
  else {
    const cv::Rect validROI(0, 0, outputValidSize.width, outputValidSize.height);

    outImage.create(fftSize), outImage.setTo(0);

    if ( smallImage.depth() == CV_32F ) {
      smallImage.copyTo(outImage(validROI));
    }
    else {
      smallImage.convertTo(outImage(validROI), CV_32F);
    }
  }
}

cv::Size c_polar_phase_correlate::computeFFTPackSize(const cv::Size & expectedFrameSize)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  const int W = getOptimalFFTSizeDown(expectedFrameSize.width);
  const int H = getOptimalFFTSizeDown(expectedFrameSize.height);
  return cv::Size(std::max(4, W), std::max(4, H));
}

bool c_polar_phase_correlate::setup(const cv::Size & expectedFrameSize, c_polar_phase_correlate_options & opts)
{
  INSTRUMENT_REGION("");

  if( expectedFrameSize.empty() ) {
    CF_ERROR("c_polar_phase_correlate: BAD expectedFrameSize specified : %dx%d",
        expectedFrameSize.width, expectedFrameSize.height);
    return false;
  }

  _fftSize = computeFFTPackSize(expectedFrameSize);
  _expectedFrameSize = expectedFrameSize;
  _gsigma1 = opts.gsigma1;
  _maxPooling = opts.maxPooling;
  _whitenSpecs = opts.whitenSpecs;

  if ( opts.gsigma2 != _gsigma2 ) {
    _gsigma2 = opts.gsigma2;
    _bp2.release();
  }

  if ( _aalpha != opts.apodization ) {
    _aalpha = opts.apodization;
    _apodizationWindow.release();
  }

  // pre-generate bandpass filter for expected frame size
  generateBandpassFilter1();

  _initialized = true;
  return true;
}

void c_polar_phase_correlate::release()
{
  _fftSize = cv::Size(0, 0);

//
//  _scaledCurrentImage.release();
//  _scaledReferenceImage.release();
//  _scaledCurrentMask.release();
//  _scaledReferenceMask.release();
//  _currentSpectrum.release();
//  _referenceSpectrum.release();
//  _crossSpectrum.release();
//  _correlationMap.release();
  _vlap1.release();
  _apodizationWindow.release();
  _initialized = false;
}

void c_polar_phase_correlate::generateBandpassFilter1()
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  // ilambda = _gsigma * (2 * sqrt(pi))
  // rho2 = (u^2 + v^2) * ilambda^2;
  // F(u, v) = rho2 * exp (-0.5 * rho2 )
  // the alternating +1 and -1 is also embedded into filter to avoid later fftSwapQudrants()

  INSTRUMENT_REGION("");

  _bp1.create(_fftSize);

  const uint8_t * filter_base = _bp1.ptr();
  const size_t filter_stride = _bp1.step;

  const int cols = _fftSize.width;
  const int rows = _fftSize.height;

  if ( _gsigma1 <= 0 ) {
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
    const double ilambda_x = (2.0 * CV_PI * _gsigma1) / cols;
    const double ilambda_y = (2.0 * CV_PI * _gsigma1) / rows;
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
          //fltp[x] = sign * rho2 * rho2 * std::exp(-rho2);
          fltp[x] = sign * rho2 * std::exp(-0.5f * rho2);
        }
      }
    });
  }

  cv::multiply(_bp1, 1. / cv::norm(_bp1, cv::NORM_L1), _bp1);

  if ( _vlap1.size() != _fftSize ) {
    _vlap1 = fftGenerateDiscreteLaplacianFilter(_fftSize, false);
  }

  if( _aalpha > 0 ) {
    const cv::Size apodizationSize((_fftSize.width + 1) / 2, (_fftSize.height + 1) / 2);
    if( _apodizationWindow.size() != apodizationSize ) {
      generateTukeyApodizationWindow(_apodizationWindow, apodizationSize, _aalpha);
    }
  }
}

void c_polar_phase_correlate::generateBandpassFilter2(const cv::Size & polarSize)
{
  const int rows = polarSize.height;
  const int cols = polarSize.width;

  _bp2.create(polarSize);

  const uint8_t * filter_base = _bp2.ptr();
  const size_t filter_stride = _bp2.step;

  if (_gsigma2 <= 0) {
    parallel_for(0, rows, [=](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        float * __restrict fltp = (float *)(filter_base + y * filter_stride);
        const float start_sign = (y & 1) ? -1.0f : 1.0f;
        for (int x = 0; x < cols; ++x) {
          fltp[x] = (x & 1) ? -start_sign : start_sign;
        }
      }
    });
  }
  else {
    const double ilambda_x = (2.0 * CV_PI * _gsigma2) / cols;
    const double ilambda_y = (2.0 * CV_PI * _gsigma2) / rows;
    const float ilambda_x2 = float(ilambda_x * ilambda_x);
    const float ilambda_y2 = float(ilambda_y * ilambda_y);

    parallel_for(0, rows, [=](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        float * __restrict fltp = (float *)(filter_base + y * filter_stride);

        const int fy = (y > rows / 2) ? (rows - y) : y;
        const float v2 = float(fy * fy * ilambda_y2);
        const float start_sign = (y & 1) ? -1.0f : 1.0f;

        for (int x = 0; x < cols; ++x) {
          const int fx = (x > cols / 2) ? (cols - x) : x;
          const float u2 = float(fx * fx * ilambda_x2);

          const float rho2 = u2 + v2;
          const float sign = (x & 1) ? -start_sign : start_sign;

          fltp[x] = sign * rho2 * std::exp(-0.5f * rho2);
        }
      }
    });
  }

  cv::multiply(_bp2, 1. / cv::norm(_bp2, cv::NORM_L1), _bp2);
}

bool c_polar_phase_correlate::setupInputFrame(cv::InputArray srcImage, cv::InputArray srcMask, Frame * frame)
{
  const cv::Size fftSize = _vlap1.size();
  if( fftSize.empty() ) {
    CF_ERROR("fftSize is empty");
    return false;
  }

  if( srcImage.empty() ) {
    CF_ERROR("input image is empty");
    return false;
  }

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

  cv::Mat smallImage;
  const double scale = 0.5;

  if( !_maxPooling ) {
    cv::resize(srcImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  }
  else {
    static const cv::Mat1b SE(3, 3, 255);
    cv::dilate(srcImage, smallImage, SE);
    cv::resize(smallImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  }

  if( !_apodizationWindow.empty() ) {
    cv::subtract(smallImage, cv::mean(smallImage), smallImage);
  }

  packScaledImageForPhaseCorreation(smallImage,
      frame->image, _fftSize,
      frame->validSize,
      frame->cropOffset);

  if ( !_apodizationWindow.empty() ) {
    const cv::Rect roi(0, 0, _apodizationWindow.cols, _apodizationWindow.rows);
    cv::multiply(frame->image(roi), _apodizationWindow, frame->image(roi));
  }

  // Call DFT with Virginie Moizan Periodic + Smooth decomposition in OpenCV CCS format
  if( !fftPPSDecompositionCCS(frame->image, _vlap1, frame->spec, cv::noArray()) ) {
    CF_ERROR("fftPPSDecompositionCCS(frame->image) fails");
    return false;
  }

  // Compute auto cross spectrum in CCS
  if ( fftAutoCrossSpectrumWeightedCCS(frame->spec, _bp1, frame->spec) <= 0 ) {
    CF_ERROR("fftAutoCrossSpectrumWeightedCCS() fails");
    return false;
  }

  cv::idft(frame->spec, frame->acm, cv::DFT_REAL_OUTPUT);
  acm2plr(frame->acm, frame->acmp, 360);

  const cv::Size polarSize = frame->acmp.size();

  if (_vlap2.size() != polarSize ) {
    _vlap2 = fftGenerateDiscreteLaplacianFilter(polarSize, false);
  }

  if (_bp2.size() != polarSize) {
    generateBandpassFilter2(polarSize);
  }

  if (!fftPPSDecompositionCCS(frame->acmp, _vlap2, frame->polarSpec, cv::noArray())) {
    CF_ERROR("fftPPSDecompositionCCS() failed for current polar frame");
    return false;
  }

  return true;
}

bool c_polar_phase_correlate::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  return setupInputFrame(currentImage, currentMask, &_currentFrame);
}

bool c_polar_phase_correlate::setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  return setupInputFrame(referenceImage, referenceMask, &_referenceFrame);
}

bool c_polar_phase_correlate::estimatePolarPeakY(const cv::Mat1f & correlationMap, double * outPeakY, double * outPeakVal)
{
  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  cv::Point maxPos;
  double measuredPeakValue = 0;
  cv::minMaxLoc(correlationMap, nullptr, &measuredPeakValue, nullptr, &maxPos);

  const int x0 = maxPos.x;
  const int y0 = maxPos.y;

  const int R = (_gsigma2 > 0) ? int(3 * _gsigma2) + 1 : 10;

  if (x0 < R || x0 >= cols - R || y0 < R || y0 >= rows - R) {
    CF_DEBUG("Bad maxPos in polar correlation: x=%d y=%d", maxPos.x, maxPos.y);
    return false;
  }

  double sw = 0;
  double sy = 0;

  const double RSCALE = -3. / (double)(R * R);
  const size_t cmap_stride = correlationMap.step;
  const uint8_t * cmap_base = correlationMap.ptr(y0);
  const double threshold = std::sqrt(1.0 / correlationMap.total());

  for (int dy = -R; dy <= R; ++dy) {
    const float * srcp = (const float *)(cmap_base + dy * cmap_stride);
    for (int dx = -R; dx <= R; ++dx) {
      const double intens = srcp[x0 + dx];
      if (intens > threshold) {
        const double v = intens - threshold;
        const double w = v * std::exp(((double)dx * dx + (double)dy * dy) * RSCALE);
        sw += w;
        sy += w * dy;
      }
    }
  }

  if (sw <= std::numeric_limits<double>::min()) {
    CF_DEBUG("Bad sw in polar spot estimation = %g", sw);
    return false;
  }

  *outPeakY = y0 + (sy / sw);
  *outPeakVal = measuredPeakValue;
  return true;
}

double c_polar_phase_correlate::compute()
{
  if (_currentFrame.acmp.empty() || _referenceFrame.acmp.empty()) {
    CF_ERROR("c_polar_phase_correlate: Frames are not initialized. Call setReferenceImage() and setCurrentImage() first.");
    return 0;
  }

//  const cv::Size polarSize = _currentFrame.acmp.size();
//  const int rows = polarSize.height;

//  if (_vlap2.size() != polarSize || _bp2.size() != polarSize) {
//    _vlap2 = fftGenerateDiscreteLaplacianFilter(polarSize, false);
//    generateBandpassFilter2(polarSize);
//  }

//  if (!fftPPSDecompositionCCS(_currentFrame.acmp, _vlap2, _currentFrame.polarSpec, cv::noArray())) {
//    CF_ERROR("fftPPSDecompositionCCS() failed for current polar frame");
//    return 0;
//  }
//
//  if (!fftPPSDecompositionCCS(_referenceFrame.acmp, _vlap2, _referenceFrame.polarSpec, cv::noArray())) {
//    CF_ERROR("fftPPSDecompositionCCS() failed for reference polar frame");
//    return 0;
//  }

  double crossEnergy = 1.0;

  if (_whitenSpecs) {
    if (!fftPhaseCorrelateWeightedCCS(_currentFrame.polarSpec, _referenceFrame.polarSpec, _bp2, _crossSpectrumPolar)) {
      CF_ERROR("fftPhaseCorrelateWeightedCCS() failed in polar compute");
      return 0;
    }
  }
  else {
    crossEnergy = fftCrossSpectrumWeightedCCS(_currentFrame.polarSpec, _referenceFrame.polarSpec, _bp2, _crossSpectrumPolar);
    if (crossEnergy <= 0) {
      CF_ERROR("fftCrossSpectrumWeightedCCS() failed in polar compute: crossEnergy=%g", crossEnergy);
      return 0;
    }
  }

  cv::idft(_crossSpectrumPolar, _correlationMapPolar,
      cv::DFT_REAL_OUTPUT);

  double peakY = 0;
  double peakVal = 0;
  if (!estimatePolarPeakY(_correlationMapPolar, &peakY, &peakVal)) {
    CF_ERROR("estimatePolarPeakY() failed to localize valid correlation spot");
    return 0;
  }

  if (_whitenSpecs) {
    _correlationScore = peakVal;
  }
  else {
    _correlationScore = peakVal / std::sqrt(crossEnergy);
  }

  double delta_pixels = peakY;
  if (delta_pixels > (double)(_currentFrame.acmp.rows / 2)) {
    delta_pixels -= (double)(_currentFrame.acmp.rows);
  }

  double delta_angle = delta_pixels * 0.5;

  return delta_angle;
}


bool serialize_polar_phase_correlate_options(c_config_setting section, bool save,
    c_polar_phase_correlate_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, gsigma1);
  SERIALIZE_OPTION(section, save, opts, apodization);
  SERIALIZE_OPTION(section, save, opts, maxPooling);
  return true;
}
