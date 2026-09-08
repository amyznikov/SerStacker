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

bool c_phase_correlate::setup(const cv::Size & expectedFrameSize, c_phase_correlate_options & opts)
{
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

  // Find the closest power of two (round mathematically to the nearest)
  // cvRound(std::log2(v)) will select the power that is closest to the target
  // Some limit from below (for example not less than 64 pixels, so that the algorithm does not degenerate)
  // Return the size as 2^powX and 2^powY

  const int downscaledW = cvRound(expectedFrameSize.width / _downscale_factor);
  const int downscaledH = cvRound(expectedFrameSize.height / _downscale_factor);
  const int powX = std::max(6, cvCeil(std::log2(downscaledW)));
  const int powY = std::max(6, cvCeil(std::log2(downscaledH)));
  _fftSize = cv::Size(1 << powX, 1 << powY);

  if( (_apodization_size = opts.apodization_size) < 1 ) {
    _apodizationLUT.clear();
  }
  else {
    // Smoothly increases from 0.0 to 1.0
    const int ksize = _apodization_size;
    _apodizationLUT.resize(ksize + 1);
    for( int i = 0; i <= ksize; ++i ) {
      const float t = float(i) / ksize;
      _apodizationLUT[i] = t * t * (3.0f - 2.0f * t);
    }
  }

  return true;
}

void c_phase_correlate::release()
{
  _fftSize = cv::Size(0, 0);
  _apodizationLUT.clear();
  _scaledCurrentImage.release();
  _scaledReferenceImage.release();
  _scaledCurrentMask.release();
  _scaledReferenceMask.release();
  _currentWindow.release();
  _referenceWindow.release();
  _currentSpectrum.release();
  _referenceSpectrum.release();
  _crossSpectrum.release();
  _correlationMap.release();
  _distmap.release();
}

void c_phase_correlate::applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask,
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

bool c_phase_correlate::setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  if( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( referenceImage.empty() ) {
    CF_ERROR("c_phase_correlate: referenceImage is empty");
    return false;
  }

  if ( referenceImage.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: referenceImage must be single-channel");
    return false;
  }

  if (!referenceMask.empty() && referenceMask.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: referenceMask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;

  const double scale = 1.0 / _downscale_factor;
  cv::resize(referenceImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  if (!referenceMask.empty()) {
    cv::resize(referenceMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  }

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      _scaledReferenceImage, _scaledReferenceMask, _fftSize,
      _referenceValidSize,
      _referenceCropOffset);

  if (_apodization_size > 0) {
    applyApodization(_scaledReferenceImage, _scaledReferenceMask,
        _referenceValidSize);
  }

  cv::dft(_scaledReferenceImage, _currentSpectrum,
      cv::DFT_REAL_OUTPUT);

  return true;
}

bool c_phase_correlate::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  if( _fftSize.empty() ) {
    CF_ERROR("c_phase_correlate: was not properly initialized, _fftSize is empty");
    return false;
  }

  if ( currentImage.empty() ) {
    CF_ERROR("c_phase_correlate: currentImage is empty");
    return false;
  }

  if ( currentImage.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: currentImage must be single-channel");
    return false;
  }

  if (!currentMask.empty() && currentMask.channels() != 1 ) {
    CF_ERROR("c_phase_correlate: currentMask must be single-channel");
    return false;
  }

  cv::Mat smallImage, smallMask;
  const double scale = 1.0 / _downscale_factor;
  cv::resize(currentImage, smallImage, cv::Size(0, 0), scale, scale, cv::INTER_AREA);
  if (!currentMask.empty()) {
    cv::resize(currentMask, smallMask, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
  }

  packScaledImageForPhaseCorreation(smallImage, smallMask,
      _scaledCurrentImage, _scaledCurrentMask, _fftSize,
      _currentValidSize,
      _currentCropOffset);

  if (_apodization_size > 0) {
    applyApodization(_scaledCurrentImage, _scaledCurrentMask,
        _currentValidSize);
  }

  cv::dft(_scaledCurrentImage, _referenceSpectrum,
      cv::DFT_REAL_OUTPUT);

  return true;
}

double c_phase_correlate::computeCorrelationMap()
{
  const int rows = _currentSpectrum.rows;
  const int cols = _currentSpectrum.cols;
  const bool is_even = (cols % 2 == 0);
  const float gsigma = float(_gsigma);
  const float norm_factor = float(1.64872127 / _gsigma);
  const float inv_cols = float(1.0 / cols);
  const float inv_rows = float(1.0f / rows);

  _crossSpectrum.create(_currentSpectrum.size());

  const uint8_t * spec1_base = _currentSpectrum.ptr();
  const size_t spec1_stride = _currentSpectrum.step;

  const uint8_t * spec2_base = _referenceSpectrum.ptr();
  const size_t spec2_stride = _referenceSpectrum.step;

  uint8_t * cross_base = _crossSpectrum.ptr();
  const size_t cross_stride = _crossSpectrum.step;

  const float inv_gsigma_sq_2 = -0.5f / (gsigma * gsigma);

  std::atomic<float> total_energy(0.0f);

  parallel_for(0, rows, [=, &total_energy](const auto & range) {
    float local_energy = 0.0f;

    for( int y = rbegin(range); y < rend(range); ++y ) {
      const float * srcp1 = (const float * )(spec1_base + y * spec1_stride);
      const float * srcp2 = (const float * )(spec2_base + y * spec2_stride);
      float * __restrict dstp = (float *)(cross_base + y * cross_stride);

      const float v = (y > rows / 2) ? (float)(rows - y) * inv_rows : (float)y * inv_rows;
      const float v_sq = v * v;

      dstp[0] = 0.0f;

      const int max_complex_idx = is_even ? (cols - 2) : (cols - 1);
      for( int x = 1; x <= max_complex_idx; x += 2 ) {
        const int fx = (x + 1) / 2;

        const float u = (float)fx * inv_cols;
        const float rho_sq = u * u + v_sq;
        const float rho = std::sqrt(rho_sq);

        const float total_gaussian = std::exp(rho_sq * inv_gsigma_sq_2);
        const float weight = norm_factor * rho * total_gaussian;

        const float a = srcp1[x];
        const float b = srcp1[x + 1];
        const float c = srcp2[x];
        const float d = srcp2[x + 1];
        const float re = a * c + b * d;
        const float im = b * c - a * d;
        const float mag = std::sqrt(re * re + im * im);
        const float sign_fx = ((fx + y) % 2 == 0) ? 1.0f : -1.0f;
        const float w = weight * sign_fx / ((mag > 0) ? mag : 1.0f);

        local_energy += (mag > 0) ? weight : 0.0f;
        dstp[x] = re * w;
        dstp[x + 1] = im * w;
      }

      if( is_even ) {
        dstp[cols - 1] = 0.0f;
      }
    }

    float current = total_energy.load(std::memory_order_relaxed);
    while (!total_energy.compare_exchange_weak(current, current + local_energy,
            std::memory_order_relaxed));
  });

  const double total_filter_energy = 2 * total_energy.load();

  cv::idft(_crossSpectrum, _correlationMap, cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);

  return total_filter_energy;
}

double c_phase_correlate::compute(cv::Vec2f & outputTranslation)
{
  if (_fftSize.empty() || _currentSpectrum.size() != _fftSize || _referenceSpectrum.size() != _fftSize ) {
    return -1.0;
  }

  cv::Point peakLoc;

  const double total_filter_energy = computeCorrelationMap();

  const cv::Point2f scaledTranslation = findSubpixelCentroid(_correlationMap, peakLoc);
  const double rawPeak = _correlationMap(peakLoc.y, peakLoc.x);
  const double normalizedResponse = (total_filter_energy > 0) ? (rawPeak / total_filter_energy) : 0.0;
  const double correlationScore = normalizedResponse * _correlationMap.size().area();

  const double dX = _referenceCropOffset.x - _currentCropOffset.x;
  const double dY = _referenceCropOffset.y - _currentCropOffset.y;
  const double scaledDx = scaledTranslation.x - (_fftSize.width / 2.0) + dX;
  const double scaledDy = scaledTranslation.y - (_fftSize.height / 2.0) + dY;
  outputTranslation[0] = float(scaledDx * _downscale_factor);
  outputTranslation[1] = float(scaledDy * _downscale_factor);

  return correlationScore;
}

cv::Point2f c_phase_correlate::findSubpixelCentroid(const cv::Mat1f & correlationMap, cv::Point & outPeakLoc)
{
  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  cv::minMaxLoc(correlationMap, nullptr, nullptr, nullptr, &outPeakLoc);

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

      float mass = std::max(0.0f, correlationMap(sampleY, sampleX));
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

bool serialize_phase_correlate_options(c_config_setting section, bool save,
    c_phase_correlate_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, downscale_factor);
  SERIALIZE_OPTION(section, save, opts, gsigma);
  SERIALIZE_OPTION(section, save, opts, apodization_size);
  return true;
}

#if 0
cv::Point2f c_phase_correlate::findSubpixelCentroid2(const cv::Mat1f & correlationMap, cv::Point & outPeakLoc, float & outSecondPeakVal)
{
  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  double maxVal1;
  cv::minMaxLoc(correlationMap, nullptr, &maxVal1, nullptr, &outPeakLoc);

  const float peakWidthMultiplier = 1.75f;

  const int exclude_r = std::max(3, static_cast<int>(std::ceil((3.0f * peakWidthMultiplier) / (2.0f * CV_PI * _gsigma))));

  float maxVal2 = -1.0f;
  cv::Point peak2Loc(0, 0);

  for (int y = 0; y < rows; ++y) {
    const float* rowCorr = correlationMap.ptr<float>(y);

    int distY1 = std::abs(y - outPeakLoc.y);
    int distY2 = rows - distY1;
    bool in_exclude_y = (std::min(distY1, distY2) <= exclude_r);

    for (int x = 0; x < cols; ++x) {
      if (in_exclude_y) {
        int distX1 = std::abs(x - outPeakLoc.x);
        int distX2 = cols - distX1;
        if (std::min(distX1, distX2) <= exclude_r) {
          continue; // Это склон первого пика, игнорируем его
        }
      }

      float val = rowCorr[x];
      if (val > maxVal2) {
        maxVal2 = val;
        peak2Loc = cv::Point(x, y);
      }
    }
  }

  outSecondPeakVal = maxVal2;

  double sumMass = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;
  const int r = 2;

  for( int dy = -r; dy <= r; ++dy ) {
    int sampleY = outPeakLoc.y + dy;
    if( sampleY < 0 )  sampleY += rows;
    if( sampleY >= rows ) sampleY -= rows;

    for( int dx = -r; dx <= r; ++dx ) {
      int sampleX = outPeakLoc.x + dx;
      if( sampleX < 0 )  sampleX += cols;
      if( sampleX >= cols ) sampleX -= cols;

      float mass = std::max(0.0f, correlationMap(sampleY, sampleX));
      sumMass += mass;
      sumX += static_cast<double>(outPeakLoc.x + dx) * mass;
      sumY += static_cast<double>(outPeakLoc.y + dy) * mass;
    }
  }

  double subX = (sumMass > 0.0) ? (sumX / sumMass) : static_cast<double>(outPeakLoc.x);
  double subY = (sumMass > 0.0) ? (sumY / sumMass) : static_cast<double>(outPeakLoc.y);

  return cv::Point2f(float(subX), float(subY));
}

#endif


