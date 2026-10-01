/*
 * c_phase_correlate.cc
 *
 *  Created on: Sep 7, 2026
 *      Author: amyznikov
 */
#include "c_phase_correlate.h"
#include <opencv2/geometry.hpp>
#include <core/proc/run-loop.h>
#include <core/proc/fft.h>
#include <core/debug.h>

/**
 * @brief Prepares and aligns a downscaled image and its optional mask into fixed-size FFT buffers.
 * @details Extracts a centered Region of Interest (ROI) from the source image if its dimensions
 *          exceed the target FFT grid size. It then handles zero-padding (if the input is smaller
 *          than @p fftSize), type conversion to floating-point representation (`CV_32F`), and
 *          thresholding of the mask matrix down to standard binary format (`CV_8U`).
 *
 * @param[in]  srcImage        Source downscaled image matrix.
 * @param[in]  srcMask         Source optional alignment mask matrix.
 * @param[out] outImage        Output zero-padded floating-point matrix resized to fit @p fftSize.
 * @param[out] outMask         Output binary mask matrix resized to fit @p fftSize (filled with 255 if @p srcMask is empty).
 * @param[in]  fftSize         Target fixed layout dimensions optimal for Fast Fourier Transform operations.
 * @param[out] outputValidSize The actual unpadded dimensions of the processed frame data within the padding window.
 * @param[out] cropOffset      Pixel displacement offset from the original frame center if cropping was applied.
 */
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

static int getOptimalFFTSizeDownDiv4(int size)
{
  size = size - (size % 4);
  int sopt = cv::getOptimalDFTSize(size);

  while (sopt > 0 && ((sopt > size) || (sopt % 4 != 0) || (cv::getOptimalDFTSize(sopt / 2) != sopt / 2))) {
    size -= 4;
    sopt = cv::getOptimalDFTSize(size);
  }

  return sopt;
}

// Return crossEnergy scale
static double computeCrossSpectrum(cv::InputArray currentSpectrum, cv::InputArray referenceSpectrum,
    const cv::Mat1f & bandpassFilter, cv::OutputArray crossSpectrum, bool whiten_specs)
{
  INSTRUMENT_REGION("");
  if( whiten_specs ) {

    const bool fOK =
        fftCrossSpectrumPhaseCorrelateWeightedCCS(currentSpectrum, referenceSpectrum,
            bandpassFilter, crossSpectrum);
    if( fOK ) {
      return 1;
    }

    CF_ERROR("fftCrossSpectrumPhaseCorrelateWeightedCCS() fails");
  }
  else {
    const double crossEnergy =
        fftCrossSpectrumWeightedCCS(currentSpectrum, referenceSpectrum,
            bandpassFilter, crossSpectrum);

    if( crossEnergy > 0 ) {
      return 1.0 / std::sqrt(crossEnergy);
    }

    CF_ERROR("fftCrossSpectrumWeightedCCS() fails: Energy=%g", crossEnergy);
  }

  return -1;
}

static void updateMultiROICorrelationMap(bool initialize, cv::InputArray _currentRoiMap,
    cv::InputOutputArray _cumulativeMap, double _scale)
{
  INSTRUMENT_REGION("");
  const cv::Mat roiMap = _currentRoiMap.getMat();
  const uint8_t * roimap_base = roiMap.ptr();
  const size_t roimap_stride = roiMap.step;

  const int rows = roiMap.rows;
  const int cols = roiMap.cols;
  const float scale = float(_scale);

  if ( _cumulativeMap.size() != roiMap.size() ) {
    initialize = true; // force initialize
  }

  if ( initialize ) {
    _cumulativeMap.create(rows, cols, CV_32FC1);

    cv::Mat & cMap = _cumulativeMap.getMatRef();
    const uint8_t * cmap_base = cMap.ptr();
    const size_t cmap_stride = cMap.step;

    parallel_for(0, rows, [=](const auto & range) {
      for ( int y = rbegin(range); y < rend(range); ++y ) {
        const float * rmap = (const float *)(roimap_base + y * roimap_stride);
        float * __restrict cmap = (float *)(cmap_base + y * cmap_stride);
        for ( int x = 0; x < cols; ++x ) {
          cmap[x] = std::max(0.f, scale * rmap[x]);
        }
      }
    });
  }
  else {
    cv::Mat & cMap = _cumulativeMap.getMatRef();
    const uint8_t * cmap_base = cMap.ptr();
    const size_t cmap_stride = cMap.step;

    parallel_for(0, rows, [=](const auto & range) {
      for ( int y = rbegin(range); y < rend(range); ++y ) {
        const float * rmap = (const float *)(roimap_base + y * roimap_stride);
        float * __restrict cmap = (float *)(cmap_base + y * cmap_stride);
        for ( int x = 0; x < cols; ++x ) {
          const float v = cmap[x];
          cmap[x] = v * std::max(0.f, scale * rmap[x]);
        }
      }
    });
  }
}


cv::Size c_phase_correlate::computeFFTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor, bool multi_roi)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  const int W = cvRound(expectedFrameSize.width / downscaleFactor);
  const int H = cvRound(expectedFrameSize.height / downscaleFactor);
  const int downscaledW = multi_roi ? getOptimalFFTSizeDownDiv4(W) : getOptimalFFTSizeDown(W);
  const int downscaledH = multi_roi ? getOptimalFFTSizeDownDiv4(H) : getOptimalFFTSizeDown(H);
  return cv::Size(std::max(4, downscaledW), std::max(4, downscaledH));
}

bool c_phase_correlate::setup(const cv::Size & expectedFrameSize, c_phase_correlate_options & opts)
{
  INSTRUMENT_REGION("");

  if( expectedFrameSize.empty() ) {
    CF_ERROR("c_phase_correlate: BAD expectedFrameSize specified : %dx%d",
        expectedFrameSize.width, expectedFrameSize.height);
    return false;
  }

  if( (_downscale_factor = opts.downscale_factor) < 1 ) {
    CF_ERROR("c_phase_correlate: BAD downscale_factor specified : %g. Must be >= 1",
        opts.downscale_factor);
    return false;
  }

  const cv::Size totalPackSize = computeFFTPackSize(expectedFrameSize, _downscale_factor, opts.multi_roi);
  if ( !opts.multi_roi ) {
    _fftSize = totalPackSize;
  }
  else {
    _fftSize.width = totalPackSize.width / 2;
    _fftSize.height = totalPackSize.height / 2;
  }

  _expectedFrameSize = expectedFrameSize;
  _gsigma = opts.gsigma;
  _csigma = opts.csigma;
  _calpha = opts.calpha;
  _whiten_specs = opts.whiten_specs;
  _multi_roi = opts.multi_roi;

  if ( _aalpha != opts.apodization ) {
    _aalpha = opts.apodization;
    _apodizationWindow.release();
  }

  // pre-generate bandpass filter for expected frame size
  generateFilters();

  _initialized = true;
  return true;
}

void c_phase_correlate::release()
{
  _fftSize = cv::Size(0, 0);
  _scaledCurrentImage.release();
  _scaledReferenceImage.release();
  _scaledCurrentMask.release();
  _scaledReferenceMask.release();
  _currentSpectrum.release();
  _referenceSpectrum.release();
  _crossSpectrum.release();
  _correlationMap.release();
  _vlapFilter.release();
  _apodizationWindow.release();
  _initialized = false;
}

/**
 * @brief Pre-calculates the frequency bandpass weighting filter matrix based on pipeline options.
 * @details Generates a 2D bandpass filter grid directly mapped to the @ref _fftSize.
 *          The method utilizes multi-threaded execution via OpenMP/TBB (`parallel_for`) and embeds
 *          spatial chessboard sign-alternation (`((x + y) & 1) ? -1 : 1`) to inherently shift
 *          the spectrum origin to the center, bypassing the performance overhead of an explicit
 *          `fftShift` loop.
 *
 *          If `@ref _gsigma <= 0`, a fallback high-pass/alternating matrix is generated. Otherwise,
 *          a Rayleigh-like frequency distribution curve is evaluated. If `@ref _csigma > 0` and
 *          `@ref _calpha > 0`, an additional inverse cross-filter mask for mask-edge or blur
 *          deconvolution is combined with the map. The final matrix is fully normalized via the L1 norm.
 *
 */
void c_phase_correlate::generateFilters()
{
  INSTRUMENT_REGION("");
  // ilambda = _gsigma / 0.283
  // rho2 = (u^2 + v^2) * ilambda^2;
  // F(u, v) = rho2 * exp (-rho2 )
  // the alternating +1 and -1 is also embedded into filter to avoid later fftSwapQudrants()

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
    // gsigma = 0.283 / lambda
    // 1 / lambda = gsigma/0.283
    const double ilambda = (_gsigma / 0.283);
    const float ilambda2 = float (ilambda * ilambda);

    const float icols = float (1.0 / cols);
    const float irows = float (1.0 / rows);

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);

        const int fy = (y > rows / 2) ? (rows - y) : y;
        const float v = fy * irows;
        const float v2 = v * v;

        for( int x = 0; x < cols; ++x ) {
          const int fx = (x > cols / 2) ? (cols - x) : x;
          const int sign = ((x + y) & 1) ? -1 : 1;

          const float u = fx * icols;
          const float u2 = u * u;
          const float rho2 = (u2 + v2) * ilambda2;
          fltp[x] = sign * rho2 * std::exp(-rho2);
        }
      }
    });
  }

  if ( _csigma > 0  && _calpha > 0) {
    // Embed also cross-like spectrum features suppression directly into filter instead of pixel space apodization
    fftGenerateInverseCrossFilter(_fftSize, _expectedFrameSize, _crossMask, _csigma, _calpha, false);
    cv::multiply(_bandpassFilter, _crossMask, _bandpassFilter);
  }

  cv::multiply(_bandpassFilter, 1. / cv::norm(_bandpassFilter, cv::NORM_L1),
      _bandpassFilter);

  if ( _vlapFilter.size() != _fftSize ) {
    _vlapFilter = fftGenerateDiscreteLaplacianFilter(_fftSize, false);
  }

  if ( _aalpha > 0 &&  _apodizationWindow.size() != _fftSize ) {
    generateTukeyApodizationWindow(_apodizationWindow, _fftSize, _aalpha);
  }
}

bool c_phase_correlate::setupInputImage(cv::InputArray srcImage, cv::InputArray srcMask,
    cv::Mat1f & outScaledImage, cv::Mat1b & outScaledMask,
    cv::Size & outValidSize, cv::Point & outCropOffset,
    cv::Mat1f & outSingleSpectrum,
    std::vector<cv::Mat1f> & outMultiSpectrums)
{
  INSTRUMENT_REGION("");

  if ( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( srcImage.empty() ) {
    CF_ERROR("c_phase_correlate: input image is empty");
    return false;
  }

  if ( srcImage.channels() != 1 || (!srcMask.empty() && srcMask.channels() != 1) ) {
    CF_ERROR("c_phase_correlate: input image and mask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;
  if ( std::abs(_downscale_factor - 1) < 10 * FLT_EPSILON ) {
    smallImage = srcImage.getMat();
    smallMask = srcMask.getMat();
  }
  else {
    const double scale = 1.0 / _downscale_factor;
    cv::resize(srcImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
    if (!srcMask.empty()) {
      cv::resize(srcMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
    }
  }

  const cv::Size packSize = _multi_roi ?
      cv::Size(_fftSize.width * 2, _fftSize.height * 2) :
      _fftSize;

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      outScaledImage, outScaledMask, packSize,
      outValidSize,
      outCropOffset);

  // Call DFT with Virginie Moizan Periodic + Smooth decomposition in OpenCV CCS format

  if ( !_multi_roi ) {
    if ( _apodizationWindow.size() == _fftSize ) {
      cv::multiply(_apodizationWindow, outScaledImage, outScaledImage);
    }
    fftPPSDecompositionCCS(outScaledImage, _vlapFilter, outSingleSpectrum, cv::noArray());
  }
  else {
    cv::Mat1f subImage;

    outMultiSpectrums.resize(4);

    for ( int y = 0; y < 2; ++y ) {
      for ( int x = 0; x < 2; ++x ) {
        const cv::Rect ROI(x * _fftSize.width, y * _fftSize.height, _fftSize.width, _fftSize.height);
        if ( _apodizationWindow.empty() ) {
          fftPPSDecompositionCCS(outScaledImage(ROI), _vlapFilter, outMultiSpectrums[2 * y + x], cv::noArray());
        }
        else {
          cv::multiply(_apodizationWindow, outScaledImage(ROI), subImage);
          fftPPSDecompositionCCS(subImage, _vlapFilter, outMultiSpectrums[2 * y + x], cv::noArray());
        }
      }
    }
  }

  return true;
}

bool c_phase_correlate::setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  return setupInputImage(referenceImage, referenceMask,
      _scaledReferenceImage, _scaledReferenceMask,
      _referenceValidSize, _referenceCropOffset,
      _referenceSpectrum, _referenceSpectrums);
}

bool c_phase_correlate::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  return setupInputImage(currentImage, currentMask,
      _scaledCurrentImage, _scaledCurrentMask,
      _currentValidSize, _currentCropOffset,
      _currentSpectrum, _currentSpectrums);
}

/**
 * @brief Computes phase correlation map and estimates the precise 2D translation vector.
 * @details Executes cross-spectrum phase evaluation, performs an inverse DFT, interpolates
 *          the subpixel peak, and shifts the result back to original unscaled pixel units,
 *          accounting for downscaling and crop offsets.
 *
 * @param[out] outputTranslation Resulting [dx, dy] translation vector in original pixels.
 * @return Overlap-compensated correlation quality score (PSR-like metric), or -1 on failure.
 */
double c_phase_correlate::compute(cv::Vec2f & outputTranslation)
{
  INSTRUMENT_REGION("");

  if ( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized with setup()");
    return -1;
  }

  if ( !_multi_roi ) {
    if ( _currentSpectrum.size() != _fftSize || _referenceSpectrum.size() != _fftSize ) {
      CF_ERROR("c_phase_correlate: single-ROI spectrum size mismatch");
      return -1;
    }
  }
  else {
    if ( _currentSpectrums.size() != 4 || _referenceSpectrums.size() != 4 ) {
      CF_ERROR("c_phase_correlate: multi-ROI spectrums not initialized");
      return -1;
    }
  }

  _crossEnergyScale = 1;

  if( !_multi_roi ) {

    _crossEnergyScale =
        computeCrossSpectrum(_currentSpectrum, _referenceSpectrum,
            _bandpassFilter, _crossSpectrum, _whiten_specs);

    cv::idft(_crossSpectrum, _correlationMap, cv::DFT_REAL_OUTPUT);

  }
  else {
    cv::Mat1f cSpec, cMap;

    for( size_t i = 0; i < 4; ++i ) {

      _crossEnergyScale =
          computeCrossSpectrum(_currentSpectrums[i], _referenceSpectrums[i],
              _bandpassFilter, cSpec, _whiten_specs);

      cv::idft(cSpec, cMap, cv::DFT_REAL_OUTPUT);

      updateMultiROICorrelationMap(i == 0, cMap, _correlationMap,
          _crossEnergyScale);
    }

  }

  cv::Point2f peakPos;
  cv::Point maxPos;
  const double measuredPeakValue = findSubpixelCentroid(_correlationMap, peakPos, maxPos);
  if ( !_multi_roi ) {
    _peakValue = measuredPeakValue * _crossEnergyScale;
  }
  else {
    _peakValue = std::sqrt(std::sqrt(measuredPeakValue));
  }

  const double scaledDx = peakPos.x - _fftSize.width / 2.0 + _referenceCropOffset.x - _currentCropOffset.x;
  const double scaledDy = peakPos.y - _fftSize.height / 2.0 + _referenceCropOffset.y - _currentCropOffset.y;
  outputTranslation[0] = float(-scaledDx * _downscale_factor);
  outputTranslation[1] = float(-scaledDy * _downscale_factor);



  const double kx = 1.0 - std::abs(outputTranslation[0]) / (_fftSize.width * _downscale_factor);
  const double ky = 1.0 - std::abs(outputTranslation[1]) / (_fftSize.height * _downscale_factor);
  const double area_rel = kx * ky;
  const double k_min_axis = std::min(kx, ky);
  const double dynamic_eps = std::exp(-70.0 * (k_min_axis - 0.3));
  return (_correlationScore = _peakValue / (area_rel + dynamic_eps));
}

/**
 * @brief Locate the subpixel translation peak using a non-linear 5x5 centroid window.
 * @details Finds the global integer maximum and refines its location by computing a
 *          weighted center of mass over a 5x5 neighborhood. Applies an adaptive threshold
 *          to isolate the peak from bandpass sidelobes, and utilizes a cubic weighting
 *          scheme to eliminate pixel-locking effects and ensure robustness against
 *          atmospheric turbulence or temporary image splitting.
 *
 * @param correlationMap Input 2D real-valued cross-correlation map after inverse DFT.
 * @param peakPos Output refined subpixel coordinates of the correlation peak.
 * @return The absolute peak value at the integer maximum location.
 */
double c_phase_correlate::findSubpixelCentroid(const cv::Mat1f& correlationMap, cv::Point2f & peakPos, cv::Point & maxPos) const
{
  INSTRUMENT_REGION("");

  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  int maxIdx[2] = {0, 0};
  cv::minMaxIdx(correlationMap, nullptr, nullptr, nullptr, maxIdx);

  const int R = _gsigma >= 0 ? int( _whiten_specs ?  _gsigma :  2 * _gsigma) + 1 : 2;
  const int y0 = maxIdx[0];
  const int x0 = maxIdx[1];
  maxPos.x = x0;
  maxPos.y = y0;

  if (x0 < R || x0 >= cols - R || y0 < R || y0 >= rows - R) {
    peakPos = cv::Point2f(float(x0), float(y0));
    return -1;
  }

  const float z_center = correlationMap(y0, x0);
  double sumWeights = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;

  if ( !_multi_roi ) {
    const uint8_t * cmap_base = correlationMap.ptr(y0);
    const size_t cmap_stride = correlationMap.step;

    const double T = z_center * 0.10f;
    for (int dy = -R; dy <= R; ++dy) {
      const float * __restrict srcp = (const float *)(cmap_base + dy * cmap_stride);
      for (int dx = -R; dx <= R; ++dx) {
        const double val = srcp[x0 + dx];
        if (val > T) {
          const double diff = val - T;
          const double w = diff * diff * diff;
          sumWeights += w;
          sumX += dx * w;
          sumY += dy * w;
        }
      }
    }
  }
  else {
    const uint8_t * cmap_base = correlationMap.ptr(y0);
    const size_t cmap_stride = correlationMap.step;

    const double T = z_center * 0.02f;
    for (int dy = -R; dy <= R; ++dy) {
      const float * __restrict srcp = (const float *)(cmap_base + dy * cmap_stride);
      for (int dx = -R; dx <= R; ++dx) {
        const double val = srcp[x0 + dx];
        if (val > T) {
          const double w = val - T;
          sumWeights += w;
          sumX += dx * w;
          sumY += dy * w;
        }
      }
    }
  }

  if( sumWeights > 0 ) {
    peakPos.x = float(x0 + sumX / sumWeights);
    peakPos.y = float(y0 + sumY / sumWeights);
  }
  else {
    peakPos = cv::Point2f(float(x0), float(y0));
  }

  if( _whiten_specs ) {
    // TEMPORARY EXPERIMENTAL CODE
    const double _min_alignment_quality = 0.2;
    const double E = std::sqrt(_crossEnergyScale / _fftSize.area());
    const double T = E; // z_center * 0.02f;
    const int X0 = (int) (peakPos.x);
    const int Y0 = (int) (peakPos.y);
    const int R3 = 2 * R;

    const uint8_t * cmap_base = correlationMap.ptr(Y0);
    const size_t cmap_stride = correlationMap.step;

    double sw = 0;
    double sx = 0, sy = 0;
    double sx2 = 0, sy2 = 0, sxy = 0;
    double sx3 = 0, sy3 = 0, sx2y = 0, sxy2 = 0;

    for( int dy = -R3; dy <= R3; ++dy ) {
      const float * __restrict srcp = (const float*) (cmap_base + dy * cmap_stride);
      for( int dx = -R3; dx <= R3; ++dx ) {
        const double v = srcp[X0 + dx];
        if( v > T ) {
          const double dx2 = (double) dx * dx;
          const double dy2 = (double) dy * dy;
          const double w = (v - T) * std::exp(-0.5 * (dx2 + dy2) / (_gsigma * _gsigma));

          sw += w;
          sx += w * dx;
          sy += w * dy;
          sx2 += w * dx2;
          sy2 += w * dy2;
          sxy += w * dx * dy;
          sx3 += w * dx2 * dx;
          sy3 += w * dy2 * dy;
          sx2y += w * dx2 * dy;
          sxy2 += w * dx * dy2;
        }
      }
    }

    if (sw > 1e-12) {
      const double cx = sx / sw;
      const double cy = sy / sw;

      const double mu20 = (sx2 / sw) - (cx * cx);
      const double mu02 = (sy2 / sw) - (cy * cy);
      const double mu11 = (sxy / sw) - (cx * cy);

      const double cx2 = cx * cx;
      const double cy2 = cy * cy;
      const double mu30 = (sx3 / sw) - 3.0 * (sx2 / sw) * cx + 2.0 * cx2 * cx;
      const double mu03 = (sy3 / sw) - 3.0 * (sy2 / sw) * cy + 2.0 * cy2 * cy;
      const double mu21 = (sx2y / sw) - 2.0 * (sxy / sw) * cx - (sx2 / sw) * cy + 2.0 * cx2 * cy;
      const double mu12 = (sxy2 / sw) - 2.0 * (sxy / sw) * cy - (sy2 / sw) * cx + 2.0 * cx * cy2;

      const double diff = mu20 - mu02;
      const double term = std::sqrt(diff * diff + 4.0 * mu11 * mu11);

      const double lambda_max = 0.5 * (mu20 + mu02 + term);
      const double lambda_min = 0.5 * (mu20 + mu02 - term);

      const double eccentricity = (lambda_max > 1e-12) ? std::sqrt(1.0 - (lambda_min / lambda_max)) : 0.0;
      const double angle = 0.5 * std::atan2(2.0 * mu11, diff);

      const double cos_a = std::cos(angle);
      const double sin_a = std::sin(angle);
      const double cos_a2 = cos_a * cos_a;
      const double sin_a2 = sin_a * sin_a;
      const double cos_a3 = cos_a2 * cos_a;
      const double sin_a3 = sin_a2 * sin_a;

      const double mu30_rot = mu30 * cos_a3
          + 3.0 * mu21 * cos_a2 * sin_a
          + 3.0 * mu12 * cos_a * sin_a2
          + mu03 * sin_a3;

      const double mu03_rot = -mu30 * sin_a3
          + 3.0 * mu21 * sin_a2 * cos_a
          - 3.0 * mu12 * sin_a * cos_a2
          + mu03 * cos_a3;

      const double lambda_ref = 0.1716 * (double)_gsigma * (double)_gsigma;
      const double gsigma3 = std::pow((double)_gsigma, 3.0);

      const double skew_major_stable = (gsigma3 > 1e-12) ? (mu30_rot / gsigma3) : 0.0;
      const double skew_minor_stable = (gsigma3 > 1e-12) ? (mu03_rot / gsigma3) : 0.0;

      const double def_scale = std::max(0.0, (lambda_max / lambda_ref) - 1.0);
      const double def_skew  = std::sqrt(skew_major_stable * skew_major_stable + skew_minor_stable * skew_minor_stable);

      const double q_scale = 1.0 / (1.0 + 0.4 * def_scale);
      const double q_shape = 1.0 - eccentricity;
      const double q_skew  = std::max(0.0, 1.0 - 2.0 * def_skew);

      const double alignment_quality =  q_scale * q_shape * q_skew;
      bool is_bad_align = (alignment_quality < _min_alignment_quality);

      CF_DEBUG("\n"
          "BOX: {%d, %d, %dx%d}\n"
          "_crossEnergyScale = %g E = %g T = %g\n"
          "cx = %g cy = %g\n"
          "gsigma=%g lambda_ref=%g lambda_min = %g lambda_max = %g eccentricity = %g angle = %g\n"
          "skew_minor = %g skew_major = %g\n"
          "DIAGNOSTICS: q_scale = %g, q_shape = %g, q_skew = %g\n"
          "alignment_quality = %g is_bad_align = %d\n",
          X0 - R3, Y0 - R3, 2 * R3 + 1, 2 * R3 + 1,
          _crossEnergyScale, E, T,
          cx, cy,
          _gsigma, lambda_ref, lambda_min, lambda_max, eccentricity, angle * 180 / CV_PI,
          skew_minor_stable, skew_major_stable,
          q_scale, q_shape, q_skew,
          alignment_quality, is_bad_align
          );
    }
  }

  return z_center;
}

bool serialize_phase_correlate_options(c_config_setting section, bool save,
    c_phase_correlate_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, downscale_factor);
  SERIALIZE_OPTION(section, save, opts, gsigma);
  SERIALIZE_OPTION(section, save, opts, csigma);
  SERIALIZE_OPTION(section, save, opts, calpha);
  SERIALIZE_OPTION(section, save, opts, apodization);
  SERIALIZE_OPTION(section, save, opts, whiten_specs);
  SERIALIZE_OPTION(section, save, opts, multi_roi);
  return true;
}


