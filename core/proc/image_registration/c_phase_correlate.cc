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

  _gsigma = opts.gsigma;

  _initialized = true;
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
  _currentSpectrum.release();
  _referenceSpectrum.release();
  _crossSpectrum.release();
  _correlationMap.release();
  _distmap.release();
  _initialized = false;
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

  cv::dft(_scaledReferenceImage, _referenceSpectrum,
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

  cv::dft(_scaledCurrentImage, _currentSpectrum,
      cv::DFT_REAL_OUTPUT);

  return true;
}


double c_phase_correlate::computeCorrelationMap()
{
  // The current and reference spectrums are expected in
  // CCS (Complex-Conjugate Symmetrical ) format computed
  // from real images with cv::DFT_REAL_OUTPUT flag
  //
  // The cross spectrum is weighted by bandpass filter:
  // w(f) ~ f^2 * exp(-0.5 * f^2 / sigma_f^2)
  //
  // The _gsigma is passed as the characteristic scale lambda in pixels, e.g., 10.0f
  // The normalization factor makes the filter peak w(f_max) equal to 1.0:
  //  w(f_max) = 1 / (e * lambda_max^2) -> norm_factor = e * lambda_max^2
  //  e = 2.71828183f
  //
  // Additionally the cross spectrum it is multiplied by alternating +1 and -1
  // to avoid fftSwapQuadrants() after idft()

  const int rows = _currentSpectrum.rows;
  const int cols = _currentSpectrum.cols;
  const bool is_even = (cols % 2 == 0);
  const float inv_cols = float(1.0 / cols);
  const float inv_rows = float(1.0f / rows);
  const float lambda_max = float(_gsigma);

  const bool unique_weight = (lambda_max > 0.1f);
  const float inv_gsigma2 = unique_weight ? -(lambda_max * lambda_max) : 0.0f;

  // TODO: Precompute exp(-u^2) in setup() ?
  std::vector<float> lut_exp_u(cols, 1.0f);
  if (unique_weight) {
    const int max_complex_idx = is_even ? (cols - 2) : (cols - 1);
    for (int x = 1; x <= max_complex_idx; x += 2) {
      const int fx = (x + 1) / 2;
      const float u = fx * inv_cols;
      const float exp_u = std::exp(u * u * inv_gsigma2);
      lut_exp_u[x] = exp_u;
      lut_exp_u[x + 1] = exp_u;
    }
  }

  _crossSpectrum.create(_currentSpectrum.size());

  const uint8_t * spec1_base = _currentSpectrum.ptr();
  const size_t spec1_stride = _currentSpectrum.step;

  const uint8_t * spec2_base = _referenceSpectrum.ptr();
  const size_t spec2_stride = _referenceSpectrum.step;

  uint8_t * cross_base = _crossSpectrum.ptr();
  const size_t cross_stride = _crossSpectrum.step;

  const float * exp_u = lut_exp_u.data();

  alignas(std::hardware_destructive_interference_size)
    std::atomic<float> total_energy(0.0f);

  parallel_for(0, rows, [=, &total_energy](const auto & range) {
    float local_energy = 0.0f;

    for( int y = rbegin(range); y < rend(range); ++y ) {
      const float * srcp1 = (const float * )(spec1_base + y * spec1_stride);
      const float * srcp2 = (const float * )(spec2_base + y * spec2_stride);
      float * __restrict dstp = (float *)(cross_base + y * cross_stride);

      const float v = ((y > rows / 2) ? (rows - y) : y )* inv_rows;
      const float exp_v = unique_weight ? std::exp(v * v * inv_gsigma2) : 1;
      const float v2 = v * v;

      dstp[0] = 0.0f;

      const int max_complex_idx = is_even ? (cols - 2) : (cols - 1);

      for( int x = 1; x <= max_complex_idx; x += 2 ) {
        const int fx = (x + 1) / 2;
        const float u = fx * inv_cols;
        const float u2 = u * u;
        const float rho2 = u2 + v2;
        const float gw = unique_weight ? (rho2 * exp_v * exp_u[x]) : 1;

        const float a = srcp1[x];
        const float b = srcp1[x + 1];
        const float c = srcp2[x];
        const float d = srcp2[x + 1];
        const float re = a * c + b * d;
        const float im = b * c - a * d;
        const float mag = std::sqrt(re * re + im * im);
        const float w = (mag > 0) ? gw * (((fx + y) % 2 == 0) ? 1 : -1) / mag : 1;

        local_energy += (mag > 0) ? gw * gw : 0.0f;
        dstp[x] = re * w;
        dstp[x + 1] = im * w;
      }

      if( is_even ) {
        dstp[cols - 1] = 0;
      }
    }

    float current = total_energy.load(std::memory_order_relaxed);
    while (!total_energy.compare_exchange_weak(current, current + local_energy,
            std::memory_order_relaxed));
  });

  const double total_filter_energy =
      2 * total_energy.load();

  // TODO: Estimate how many of the top rows of the spectrum contain valid data (non-zeros)
  // based on the vertical filter radius.
  // For example, if the active zone fits within the first N rows:
  //  const int nonzeroRows = std::min(rows, int(rows / 2));
  //  cv::idft(_crossSpectrum, _correlationMap, cv::DFT_REAL_OUTPUT | cv::DFT_SCALE, nonzeroRows);

  cv::idft(_crossSpectrum, _correlationMap,
      cv::DFT_REAL_OUTPUT| cv::DFT_SCALE);

  return total_filter_energy;
}

double c_phase_correlate::compute(cv::Vec2f & outputTranslation)
{
  if (_fftSize.empty() || _currentSpectrum.size() != _fftSize || _referenceSpectrum.size() != _fftSize ) {
    return -1.0;
  }

  const double total_filter_energy = computeCorrelationMap();
  const cv::Point2f scaledTranslation = findSubpixelCentroid(_correlationMap);

  const int rows = _correlationMap.rows;
  const int cols = _correlationMap.cols;

  /*
   * Estimate correlation score as a fraction of the total spectral energy
   * constructively interfered into cross-correlation spot.
   * May be not perfectly precise due to limited pixel grid resolution.
   * */
  const double Rspot = 0.45f * _gsigma;
  const int R = std::max(2, int(Rspot + 1.0));
  const double cx = scaledTranslation.x;
  const double cy = scaledTranslation.y;
  double spot_square_energy_sum = 0.0;

  // Scan the cross-correlation spot and accumulate total energy
  for( int dy = -R; dy <= R; ++dy ) {
    const int y = cvRound(cy) + dy;
    if( y < 0 || y >= rows ) {
      continue;
    }

    const float * rp = _correlationMap[y];
    const double delta_y = double(y) - cy;
    const double dy2 = delta_y * delta_y;

    for( int dx = -R; dx <= R; ++dx ) {
      const int x = cvRound(cx) + dx;
      if( x < 0 || x >= cols ) {
        continue;
      }

      const double delta_x = double(x) - cx;
      const double dist = std::sqrt(delta_x * delta_x + dy2);
      double pixel_weight = 0.0;
      if( Rspot < 1.5 ) {
        pixel_weight = (dist <= R) ? 1.0 : 0.0;
      }
      else if( dist <= Rspot - 0.5 ) {
        pixel_weight = 1.0;
      }
      else if( dist >= Rspot + 0.5 ) {
        pixel_weight = 0.0;
      }
      else {
        pixel_weight = (Rspot + 0.5 - dist);
      }

      if( pixel_weight > 0.0 ) {
        const double val = rp[x];
        spot_square_energy_sum += pixel_weight * val * val;
      }
    }
  }

  // Compensate for cv::idft scaling
  const double absolute_spot_energy = spot_square_energy_sum * rows * cols;
  const double correlationScore = (total_filter_energy > 0) ? (absolute_spot_energy / total_filter_energy) : 0.0;

  const double dX = _referenceCropOffset.x - _currentCropOffset.x;
  const double dY = _referenceCropOffset.y - _currentCropOffset.y;
  const double scaledDx = scaledTranslation.x - _fftSize.width / 2 + dX;
  const double scaledDy = scaledTranslation.y - _fftSize.height / 2 + dY;
  outputTranslation[0] = -float(scaledDx * _downscale_factor);
  outputTranslation[1] = -float(scaledDy * _downscale_factor);

//  CF_DEBUG("\ngsigma=%g Rspot=%g R=%d cx=%g cy=%g total_filter_energy=%g absolute_spot_energy=%g correlationScore=%g Tx=%g Ty=%g",
//      _gsigma, Rspot, R, cx, cy, total_filter_energy, absolute_spot_energy, correlationScore,
//      outputTranslation[0], outputTranslation[1]);

  return correlationScore;
}

cv::Point2f c_phase_correlate::findSubpixelCentroid(const cv::Mat1f& correlationMap) const
{
  const int rows = correlationMap.rows;
  const int cols = correlationMap.cols;

  int maxIdx[2] = {0, 0};
  cv::minMaxIdx(correlationMap, nullptr, nullptr, nullptr, maxIdx);

  const int y0 = maxIdx[0];
  const int x0 = maxIdx[1];

  if (x0 <= 0 || x0 >= cols - 1 || y0 <= 0 || y0 >= rows - 1) {
    return cv::Point2f(float(x0), float(y0));
  }

  // Least-squares approximation of a paraboloid over a 3x3 neighborhood
  const float z_00 = correlationMap(y0 - 1, x0 - 1); // Top-left
  const float z_10 = correlationMap(y0 - 1, x0); // Top-center
  const float z_20 = correlationMap(y0 - 1, x0 + 1); // Top-right

  const float z_01 = correlationMap(y0,     x0 - 1); // Middle-left
  const float z_11 = correlationMap(y0,     x0); // True center (peak)
  const float z_21 = correlationMap(y0,     x0 + 1); // Middle-right

  const float z_02 = correlationMap(y0 + 1, x0 - 1); // Bottom-left
  const float z_12 = correlationMap(y0 + 1, x0); // Bottom-center
  const float z_22 = correlationMap(y0 + 1, x0 + 1); // Bottom-right

  const float C = (z_20 + z_21 + z_22 - (z_00 + z_01 + z_02)) / 6.0f;
  const float D = (z_02 + z_12 + z_22 - (z_00 + z_10 + z_20)) / 6.0f;
  const float A = (z_00 + z_01 + z_02 + z_20 + z_21 + z_22) / 6.0f - (z_10 + z_11 + z_12) / 3.0f;
  const float B = (z_00 + z_10 + z_20 + z_02 + z_12 + z_22) / 6.0f - (z_01 + z_11 + z_21) / 3.0f;


  // Parabola extremum: delta = -derivative / (2 * curvature)
  // deltaX = -C / (2*A), deltaY = -D / (2*B)
  const float adaptive_thresh = std::abs(z_11) * 1e-6f;
  const float deltaX = std::abs(A) > adaptive_thresh ? -C / (2.0f * A) : 0.0f;
  const float deltaY = std::abs(B) > adaptive_thresh ? -D / (2.0f * B) : 0.0f;
  // CF_DEBUG("x0=%d y0=%d, deltaX=%g deltaY=%g", x0, y0, deltaX, deltaY );

  return cv::Point2f(float(x0 + deltaX), float(y0 + deltaY));
}


bool serialize_phase_correlate_options(c_config_setting section, bool save,
    c_phase_correlate_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, downscale_factor);
  SERIALIZE_OPTION(section, save, opts, gsigma);
  SERIALIZE_OPTION(section, save, opts, apodization_size);
  return true;
}


