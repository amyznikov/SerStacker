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

namespace {
using PeakMetrics = c_phase_correlate :: PeakMetrics;

static bool estimateSpotMetrics(const cv::Mat1f & correlationMap, double gsigma,
    double crossEnergyScale, double threshold, PeakMetrics * outMetrics)
{
  static constexpr double safe_min =
      std::numeric_limits<double>::min();

  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  cv::Point maxPos;
  double measuredPeakValue = 0;
  cv::minMaxLoc(correlationMap, nullptr, &measuredPeakValue, nullptr, &maxPos);

  const int x0 = maxPos.x, y0 = maxPos.y;
  const int R = (gsigma > 0) ? int(3 * gsigma) + 1 : 11; // FIXME? fallback  to 11 px if no band specified
  if (x0 < R || x0 >= cols - R || y0 < R || y0 >= rows - R) {
    CF_DEBUG("Bad maxPos: x=%d y=%d", maxPos.x, maxPos.y);
    return false;
  }

  double sw = 0, sx = 0, sy = 0;
  double sx2 = 0, sy2 = 0, sxy = 0;

  // Tolerate distant junk, exp(-3) ~ 0.05 at edges
  const double RSCALE = -3. / ((double) R * R);

  const uint8_t * cmap_base = correlationMap.ptr(y0);
  const size_t cmap_stride = correlationMap.step;

  for( int dy = -R; dy <= R; ++dy ) {
    const float * __restrict srcp = (const float*) (cmap_base + dy * cmap_stride);
    for( int dx = -R; dx <= R; ++dx ) {
      const double intens = srcp[x0 + dx] * crossEnergyScale;
      if( intens > threshold ) {
        const double v =  intens - threshold;
        const double dx2 = (double) dx * dx;
        const double dy2 = (double) dy * dy;
        const double w = v  * std::exp((dx2 + dy2) * RSCALE);
        sw += w;
        sx += w * dx;
        sy += w * dy;
        sx2 += w * dx2;
        sy2 += w * dy2;
        sxy += w * dx * dy;
      }
    }
  }

  if( !(sw > safe_min) ) {
    CF_DEBUG("Bad sw = %g", sw);
    return false;
  }

  const double cx = sx / sw;
  const double cy = sy / sw;
  const double mu20 = (sx2 / sw) - (cx * cx);
  const double mu02 = (sy2 / sw) - (cy * cy);
  const double mu11 = (sxy / sw) - (cx * cy);
  const double diff = mu20 - mu02;
  const double term = std::sqrt(std::max(0.0, diff * diff + 4.0 * mu11 * mu11));

  outMetrics->bbox.x = x0 - R;
  outMetrics->bbox.y = y0 - R;
  outMetrics->bbox.width = 2 * R + 1;
  outMetrics->bbox.height = 2 * R + 1;
  outMetrics->maxPos = maxPos;
  outMetrics->peakPos.x = x0 + cx;
  outMetrics->peakPos.y = y0 + cy;
  outMetrics->semiaxes.width = std::sqrt(std::max(0., 0.5 * (mu20 + mu02 + term))); // major;
  outMetrics->semiaxes.height = std::sqrt(std::max(0.,0.5 * (mu20 + mu02 - term))); //  minor
  outMetrics->measuredPeakValue = measuredPeakValue * crossEnergyScale;

  return true;
}

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

} // namespace

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
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  // ilambda = _gsigma * (2 * sqrt(pi))
  // rho2 = (u^2 + v^2) * ilambda^2;
  // F(u, v) = rho2 * exp (-rho2 )
  // the alternating +1 and -1 is also embedded into filter to avoid later fftSwapQudrants()

  INSTRUMENT_REGION("");

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
    const double ilambda_x = (2.0 * M_PI * _gsigma) / cols;
    const double ilambda_y = (2.0 * M_PI * _gsigma) / rows;
    const float ilambda_x2 = float(ilambda_x * ilambda_x);
    const float ilambda_y2 = float(ilambda_y * ilambda_y);

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);

        const int fy = (y > rows / 2) ? (rows - y) : y;
        const float v2 = float(fy * fy) * ilambda_y2;

        for( int x = 0; x < cols; ++x ) {
          const int fx = (x > cols / 2) ? (cols - x) : x;
          const int sign = ((x + y) & 1) ? -1 : 1;
          const float u2 = float(fx * fx) * ilambda_x2;
          const float rho2 = u2 + v2;
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

double c_phase_correlate::computeCorrelationMap()
{
  double crossEnergy = 1;

  if ( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized with setup()");
    return -1;
  }

  if( !_multi_roi ) {
    if ( _currentSpectrum.size() != _fftSize || _referenceSpectrum.size() != _fftSize ) {
      CF_ERROR("c_phase_correlate: single-ROI spectrum size mismatch");
      return -1;
    }

    if( _whiten_specs ) {
      const bool fOK =
          fftPhaseCorrelateWeightedCCS(_currentSpectrum, _referenceSpectrum,
              _bandpassFilter, _crossSpectrum);
      if( !fOK ) {
        CF_ERROR("fftCrossSpectrumPhaseCorrelateWeightedCCS() fails");
        return -1;
      }
    }
    else {
      crossEnergy =
          fftCrossSpectrumWeightedCCS(_currentSpectrum, _referenceSpectrum,
              _bandpassFilter, _crossSpectrum);

      if( crossEnergy <= 0 ) {
        CF_ERROR("fftCrossSpectrumWeightedCCS() fails");
        return -1;
      }
    }

    cv::idft(_crossSpectrum, _correlationMap, cv::DFT_REAL_OUTPUT);
  }
  else {
    if ( _currentSpectrums.size() != 4 || _referenceSpectrums.size() != 4 ) {
      CF_ERROR("c_phase_correlate: multi-ROI spectrums not initialized");
      return -1;
    }

    cv::Mat1f cSpec, cMap;
    double roiCrossEnergy = 1;

    for( size_t i = 0; i < 4; ++i ) {
      if( _whiten_specs ) {
        const bool fOK =
            fftPhaseCorrelateWeightedCCS(_currentSpectrums[i], _referenceSpectrums[i],
                _bandpassFilter, cSpec);
        if( !fOK ) {
          CF_ERROR("fftCrossSpectrumPhaseCorrelateWeightedCCS() fails");
          return -1;
        }
      }
      else {
        roiCrossEnergy =
            fftCrossSpectrumWeightedCCS(_currentSpectrums[i], _referenceSpectrums[i],
                _bandpassFilter, cSpec);

        if( roiCrossEnergy <= 0 ) {
          CF_ERROR("fftCrossSpectrumWeightedCCS() fails");
          return -1;
        }
      }

      cv::idft(cSpec, cMap, cv::DFT_REAL_OUTPUT);
      updateMultiROICorrelationMap(i == 0, cMap, _correlationMap, 1.0 / std::sqrt(roiCrossEnergy));
    }
  }

  return crossEnergy;
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

  const double crossEnergy = computeCorrelationMap();
  if ( !(crossEnergy > 0) ) {
    CF_ERROR("computeCorrelationMap() fails: crossEnergy=%g", crossEnergy);
    return -1;
  }

  const double crossEnergyScale =
      _multi_roi ? 1. / std::sqrt(std::sqrt(crossEnergy)) :
          1. / std::sqrt(crossEnergy);

  const double theshold =
      _multi_roi ? 0 : std::sqrt(1.0 / _fftSize.area());

  const double r0 =
      _multi_roi ? 0.5 * _gsigma : _gsigma;

  if( !estimateSpotMetrics(_correlationMap, r0, crossEnergyScale, theshold, &_peakMetrics) ) {
    CF_ERROR("estimateSpotFirstAndSecondOrderMetrics() fails");
    return -1;
  }

  const double shift_x = std::abs(_peakMetrics.maxPos.x - _fftSize.width / 2);
  const double shift_y = std::abs(_peakMetrics.maxPos.y - _fftSize.height / 2);
  const double correction = _fftSize.area() / ((_fftSize.width - shift_x) * (_fftSize.height - shift_y));
  if ( !_multi_roi ) {
    _peakMetrics.correctedPeakValue = _peakMetrics.measuredPeakValue * correction;
  }
  else {
    _peakMetrics.correctedPeakValue = std::sqrt(std::sqrt(_peakMetrics.measuredPeakValue)) * correction;
  }

  const double scaledDx = _peakMetrics.peakPos.x - _fftSize.width / 2 + _referenceCropOffset.x - _currentCropOffset.x;
  const double scaledDy = _peakMetrics.peakPos.y - _fftSize.height / 2 + _referenceCropOffset.y - _currentCropOffset.y;
  outputTranslation[0] = float(-scaledDx * _downscale_factor);
  outputTranslation[1] = float(-scaledDy * _downscale_factor);

  return (_correlationScore = _peakMetrics.correctedPeakValue);
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


