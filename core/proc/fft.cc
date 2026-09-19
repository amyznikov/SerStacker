/*
 * fft.cc
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#include "fft.h"
#include <core/proc/run-loop.h>
#include <core/debug.h>

static int fftFindOptimalSize(int srcSize, int psfRadius, bool forceEvenSize)
{
  const int mininalSize = srcSize + 2 * psfRadius;
  int w = cv::getOptimalDFTSize(srcSize);

  if ( !forceEvenSize ) {
    while( w < mininalSize ) {
      w = cv::getOptimalDFTSize(w + 1);
    }
  }
  else {
    while ((w < mininalSize) || (w & 0x1)) {
      w = cv::getOptimalDFTSize(w + 1);
    }
  }

  return w;
}

cv::Size fftGetOptimalSize(const cv::Size & srcSize, cv::Size psfRadius, cv::Rect * roirc, bool forceEvenSize)
{
  const cv::Size fftSize(fftFindOptimalSize(srcSize.width, psfRadius.width, forceEvenSize),
      fftFindOptimalSize(srcSize.height, psfRadius.height, forceEvenSize));

  if( roirc ) {
    const int border_top = (fftSize.height - srcSize.height) / 2;
    const int border_left = (fftSize.width - srcSize.width) / 2;
    *roirc = cv::Rect(border_left, border_top, srcSize.width, srcSize.height);
  }

  return fftSize;
}

// Adjust rectangular ROI of the planet to an optimal square shape for FFT
cv::Rect fftGetOptimalSquaredROI(const cv::Size & imageSize, const cv::Rect & rawROI)
{
  const int centerX = rawROI.x + rawROI.width / 2;
  const int centerY = rawROI.y + rawROI.height / 2;
  const int maxSide = std::max(rawROI.width, rawROI.height);

  // Search for the optimal size strictly a multiple of 2
  int optimalSide = cv::getOptimalDFTSize(maxSide);
  while (optimalSide % 2 != 0) {
    optimalSide = cv::getOptimalDFTSize(optimalSide + 1);
  }

  // Protect against exceeding the physical frame dimensions
  if( optimalSide > imageSize.width || optimalSide > imageSize.height ) {
    const int absoluteMaxSide = std::min(imageSize.width, imageSize.height);
    int safeSide = absoluteMaxSide;
    while (safeSide > 0) {
      // Find an even size that is optimal for DFT
      if( safeSide % 2 == 0 && cv::getOptimalDFTSize(safeSide) == safeSide ) {
        break;
      }
      --safeSide;
    }
    optimalSide = safeSide;
  }

  // Safe position of a square with an even size
  int newX = centerX - optimalSide / 2;
  if (newX + optimalSide > imageSize.width) {
    newX = imageSize.width - optimalSide;
  }
  if (newX < 0) {
    newX = 0;
  }

  int newY = centerY - optimalSide / 2;
  if (newY + optimalSide > imageSize.height) {
    newY = imageSize.height - optimalSide;
  }
  if (newY < 0) {
    newY = 0;
  }

  return cv::Rect(newX, newY, optimalSide, optimalSide);
}

bool fftCopyMakeBorder(cv::InputArray src, cv::OutputArray dst, const cv::Size & fftSize,
    cv::Rect * outrc, cv::BorderTypes borderType)
{
  INSTRUMENT_REGION("");
  const cv::Size src_size = src.size();

  if ( fftSize.width < src_size.width || fftSize.height < src_size.height ) {
    CF_ERROR("Invalid argument: fftSize (%dx%d) must be >= src.size() (%dx%d)",
        fftSize.width, fftSize.height, src_size.width, src_size.height );
    return false;
  }

  if ( src_size.width == fftSize.width && src_size.height == fftSize.height ) {
    const cv::Mat srcm = src.getMat();
    const cv::Mat dstm = dst.getMat();
    if ( srcm.data != dstm.data || srcm.step != dstm.step ) {
      src.copyTo(dst);
    }
    if ( outrc ) {
      *outrc = cv::Rect(0, 0, src_size.width, src_size.height);
    }
  }
  else {
    const int border_top = (fftSize.height - src_size.height) / 2;
    const int border_bottom = (fftSize.height - src_size.height - border_top);
    const int border_left = (fftSize.width - src_size.width ) / 2;
    const int border_right = (fftSize.width - src_size.width - border_left);
    cv::copyMakeBorder(src, dst, border_top, border_bottom, border_left, border_right, borderType);
    if ( outrc ) {
      * outrc = cv::Rect(border_left, border_top, src_size.width, src_size.height);
    }
  }

  return true;
}

bool fftImageToSpectrum(cv::InputArray _src, cv::OutputArray _dst, const cv::Size & fftSize, bool centerDC)
{
  if( _src.type() != CV_32FC1 && _src.type() != CV_32FC2  ) {
    CF_ERROR("Invalid argument: CV_32FC1 or CV_32FC2 input image expected");
    return false;
  }

  cv::Mat src_padded;
  if( fftSize.empty() || _src.size() == fftSize ) {
    src_padded = _src.getMat();
  }
  else {
    fftCopyMakeBorder(_src, src_padded, fftSize);
  }

  cv::dft(src_padded, _dst, cv::DFT_COMPLEX_OUTPUT);
  if( centerDC ) {
    fftSwapQuadrants(_dst.getMatRef());
  }

  return true;
}

bool fftImageToSpectrum(cv::InputArray _src, std::vector<cv::Mat2f> & complex_channels,
    const cv::Size & fftSize, bool centerDC)
{
  if( _src.depth() != CV_32F ) {
    CF_ERROR("Invalid argument: CV_32F input image expected");
    return false;
  }

  cv::Mat src_padded;
  if( fftSize.empty() ||  _src.size() == fftSize ) {
    src_padded = _src.getMat();
  }
  else {
    fftCopyMakeBorder(_src, src_padded, fftSize);
  }

  const int cn = _src.channels();
  complex_channels.resize(cn);

  for( int i = 0; i < cn; ++i ) {
    cv::dft(src_padded, complex_channels[i], cv::DFT_COMPLEX_OUTPUT);
    if( centerDC ) {
      fftSwapQuadrants(complex_channels[i]);
    }
  }

  return true;
}

void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels, cv::OutputArray dst)
{
  const int cn = complex_channels.size();

  if ( cn == 1 ) {
    cv::idft(complex_channels[0], dst, cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
  }
  else {
    cv::Mat channels[cn];

    for ( int i = 0; i < cn; ++i ) {
      cv::idft(complex_channels[i], channels[i], cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
    }

    cv::merge(channels, cn, dst);
  }
}

void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels,
    cv::OutputArray dst,
    const cv::Rect & rc)
{
  if ( rc.empty() ) {
    fftImageFromSpectrum(complex_channels, dst);
  }
  else {
    cv::Mat tmp;
    fftImageFromSpectrum(complex_channels, tmp);
    dst.move(tmp = tmp(rc));
  }
}

void fftSwapQuadrants(cv::InputArray _src, cv::OutputArray _dst)
{
  const cv::Mat src = _src.getMat();
  if (src.empty()) {
    _dst.release();
    return;
  }

  cv::Mat dst(src.size(), src.type());

  const int cx = (src.cols + 1) >> 1;
  const int cy = (src.rows + 1) >> 1;
  const int inv_cx = src.cols - cx;
  const int inv_cy = src.rows - cy;

  cv::Mat src_q0(src, cv::Rect(0, 0, cx, cy));
  cv::Mat src_q1(src, cv::Rect(cx, 0, inv_cx, cy));
  cv::Mat src_q2(src, cv::Rect(0, cy, cx, inv_cy));
  cv::Mat src_q3(src, cv::Rect(cx, cy, inv_cx, inv_cy));

  cv::Mat dst_q0(dst, cv::Rect(inv_cx, inv_cy, cx, cy));  // q0 -> q3
  cv::Mat dst_q1(dst, cv::Rect(0, inv_cy, inv_cx, cy));   // q1 -> q2
  cv::Mat dst_q2(dst, cv::Rect(inv_cx, 0, cx, inv_cy)); // q2 -> q1
  cv::Mat dst_q3(dst, cv::Rect(0, 0, inv_cx, inv_cy));  // q3 -> q0

  src_q0.copyTo(dst_q0);
  src_q1.copyTo(dst_q1);
  src_q2.copyTo(dst_q2);
  src_q3.copyTo(dst_q3);

  _dst.move(dst);
}


void fftSwapQuadrants(cv::InputOutputArray _spec)
{
  const cv::Mat src = _spec.getMat();
  if (src.empty()) {
    return;
  }

  cv::Mat dst(src.size(), src.type());

  const int cx = (src.cols + 1) >> 1;
  const int cy = (src.rows + 1) >> 1;
  const int inv_cx = src.cols - cx;
  const int inv_cy = src.rows - cy;

  cv::Mat src_q0(src, cv::Rect(0, 0, cx, cy));
  cv::Mat src_q1(src, cv::Rect(cx, 0, inv_cx, cy));
  cv::Mat src_q2(src, cv::Rect(0, cy, cx, inv_cy));
  cv::Mat src_q3(src, cv::Rect(cx, cy, inv_cx, inv_cy));

  cv::Mat dst_q0(dst, cv::Rect(inv_cx, inv_cy, cx, cy));  // q0 -> q3
  cv::Mat dst_q1(dst, cv::Rect(0, inv_cy, inv_cx, cy));   // q1 -> q2
  cv::Mat dst_q2(dst, cv::Rect(inv_cx, 0, cx, inv_cy)); // q2 -> q1
  cv::Mat dst_q3(dst, cv::Rect(0, 0, inv_cx, inv_cy));  // q3 -> q0

  src_q0.copyTo(dst_q0);
  src_q1.copyTo(dst_q1);
  src_q2.copyTo(dst_q2);
  src_q3.copyTo(dst_q3);

  _spec.move(dst);
}

/* Power = Re^2 + Im^2 */
bool fftSpectrumPower(cv::InputArray _src, cv::OutputArray _dst)
{
  if ( _src.type() != CV_32FC2 ) {
    CF_ERROR("Invalid argument: CV_32FC2 input image expected ");
    return false;
  }

  if ( _dst.fixedType() && _dst.type() != CV_32FC1 ) {
    CF_ERROR("Invalid fixed type output argument: CV_32FC1 output image expected ");
    return false;
  }

  const cv::Mat2f src = _src.getMat();
  const cv::Size size = _src.size();
  _dst.create(size, CV_32FC1);
  cv::Mat1f dst = _dst.getMatRef();

  parallel_for(0, size.height, [&, size](const auto & range) {
    for ( int y = rbegin(range); y < rend(range); ++y ) {
      const float * srcp = (const float * )src[y];
      float * __restrict dstp = dst[y];
      for ( int x = 0; x < size.width; ++x, srcp += 2 ) {
        * dstp ++ = srcp[0] * srcp[0] + srcp[1] * srcp[1];
      }
    }
  });

  return true;
}

/* Module = sqrt(Re^2 + Im^2) */
bool fftSpectrumModule(cv::InputArray _src, cv::OutputArray _dst)
{
  if ( _src.type() != CV_32FC2 ) {
    CF_ERROR("Invalid argument: CV_32FC2 input image expected ");
    return false;
  }

  if ( _dst.fixedType() && _dst.type() != CV_32FC1 ) {
    CF_ERROR("Invalid fixed type output argument: CV_32FC1 output image expected ");
    return false;
  }

  std::vector<cv::Mat> planes;
  cv::split(_src, planes);
  cv::magnitude(planes[0], planes[1], _dst);
  return true;
}

bool fftSpectrumPhase(cv::InputArray _src, cv::OutputArray _dst )
{
  if ( _src.type() != CV_32FC2 ) {
    CF_ERROR("Invalid argument: CV_32FC2 input image expected ");
    return false;
  }

  if ( _dst.fixedType() && _dst.type() != CV_32FC1 ) {
    CF_ERROR("Invalid output argument: CV_32FC1 output image expected ");
    return false;
  }

  const cv::Mat2f src = _src.getMat();

  cv::Mat1f tmp;
  cv::Mat1f dst;
  if ( src.data != _dst.getMatRef().data ) {
    _dst.create(src.size(), CV_32F);
    dst = _dst.getMatRef();
  }
  else {
    tmp.create(src.size());
    dst = tmp;
  }

  parallel_loop(0, src.rows, [&src, &dst](int y) {
    for ( int x = 0; x < src.cols; ++x ) {
      dst[y][x] = std::atan2(src[y][x][1], src[y][x][0]);
    }
  });

  if ( !tmp.empty() ) {
    _dst.move(tmp);
  }

  return true;
}




bool fftSpectrumToPolar(const cv::Mat & src, cv::Mat & magnitude, cv::Mat & phase)
{
  if ( src.channels() != 2 || src.depth() != CV_32F ) {
    CF_DEBUG("invalid arg: FP32 2-channel input image expected");
    return false;
  }

  const cv::Mat2f csrc = src;

  magnitude.create(src.size(), CV_32F);
  cv::Mat1f cmag = magnitude;

  phase.create(src.size(), CV_32F);
  cv::Mat1f cphase = phase;

  parallel_loop(0, src.rows, [&](int y) {
    for ( int x = 0; x < csrc.cols; ++x ) {
      cmag[y][x] = sqrt(csrc[y][x][0]*csrc[y][x][0] + csrc[y][x][1]*csrc[y][x][1] );
      cphase[y][x] = atan2(csrc[y][x][1], csrc[y][x][0]);
    }
  });

  return true;
}

void fftSpectrumToPolar(cv::Mat2f & spec)
{
  static const float safety_thresh = std::sqrt(std::numeric_limits<float>::min());
  static constexpr float minmag = std::numeric_limits<float>::min();

  parallel_for(0, spec.rows, [&](const auto & range) {
    for ( int y = rbegin(range); y < rend(range); ++y ) {
      float * __restrict sp = (float * )spec[y];
      for ( int x = 0; x < spec.cols; ++x, sp += 2 ) {
        const float re = sp[0];
        const float im = sp[1];
        float mag = 0, phase = 0;
        if (std::abs(re) > safety_thresh || std::abs(im) > safety_thresh) {
          if ((mag = std::sqrt(re * re + im * im)) > minmag ) {
            phase = std::atan2(im, re);
          }
        }
        sp[0] = mag;
        sp[1] = phase;
      }
    }
  });
}

bool fftSpectrumToPolar(cv::InputArray spectrumCart, cv::OutputArray spectrumPolar)
{
  if( spectrumCart.empty() || spectrumCart.type() != CV_32FC2 ) {
    CF_ERROR("Invalid argument: CV_32FC2 input spectum is expected");
    return false;
  }

  const cv::Mat2f cart = spectrumCart.getMat();

  spectrumPolar.create(cart.size(), CV_32FC2);
  cv::Mat2f polar = spectrumPolar.getMatRef();

  static const float safety_thresh = std::sqrt(std::numeric_limits<float>::min());
  static constexpr float minmag = std::numeric_limits<float>::min();

  parallel_for(0, cart.rows, [&](const auto & range) {
    for ( int y = rbegin(range); y < rend(range); ++y ) {
      const float * cartp = (const float * )cart[y];
      float * __restrict polarp = (float * )polar[y];

      for ( int x = 0; x < cart.cols; ++x, cartp += 2, polarp += 2 ) {
        const float re = cartp[0];
        const float im = cartp[1];
        float mag = 0, phase = 0;
        if (std::abs(re) > safety_thresh || std::abs(im) > safety_thresh) {
          if ((mag = std::sqrt(re * re + im * im)) > minmag ) {
            phase = std::atan2(im, re);
          }
        }
        polarp[0] = mag;
        polarp[1] = phase;
      }
    }
  });

  return true;
}

bool fftSpectrumFromPolar(const cv::Mat & magnitude, const cv::Mat & phase, cv::Mat & dst )
{
  const cv::Mat1f cmag = magnitude;
  const cv::Mat1f cphase = phase;

  dst.create(cmag.size(), CV_32FC2);
  cv::Mat2f cdst = dst;

  parallel_loop(0, cdst.rows, [&](int y) {
    for ( int x = 0; x < cdst.cols; ++x ) {
      const float sa = std::sin(cphase[y][x]);
      const float ca = std::cos(cphase[y][x]);
      cdst[y][x][0] = cmag[y][x] * ca;
      cdst[y][x][1] = cmag[y][x] * sa;
    }
  });

  return true;
}

bool fftRadialProfile(const cv::Mat1f & spectrumModule, cv::Mat1f & outputProfile)
{
  // Include corners
  // The max dimensionless radius at the corner of the frame is sqrt(1^2 + 1^2) = sqrt(2)
  const int cx = spectrumModule.cols / 2;
  const int cy = spectrumModule.rows / 2;
  const float R = std::sqrt(cx * cx + cy * cy);
  const int numBins = std::max(1, int(R));

  std::vector<float> radialSum(numBins, 0.0f);
  std::vector<float> radialCount(numBins, 0.0f);

  const float scaleX = float(1. / cx);
  const float scaleY = float(1. / cy);
  const float binScale = float(numBins * M_SQRT1_2);

  for( int y = 0; y <= cy; ++y ) {
    const float dy = (y - cy) * scaleY;
    const float dy2 = dy * dy;

    const float * srcp = spectrumModule[y];
    const int xmax = (y == cy) ? (cx + 1) : spectrumModule.cols;
    for( int x = 0; x < xmax; ++x ) {
      const float dx = (x - cx) *  scaleX;
      const float dx2 = dx * dx;

      // Dimensionless radius of the ellipse:
      // 0.0 at the center, 1.0 at the sides of the matrix, ~1.414 at the corners
      const float r = std::sqrt(dx2 + dy2);
      const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);
      const int w = (y < cy ? 2 : (x == cx ? 1 : 2));
      radialSum[bin] += srcp[x] * w;
      radialCount[bin] += w;
    }
  }

  outputProfile.create(1, numBins);
  float * __restrict dstp = outputProfile[0];
  for( int i = 0; i < numBins; ++i ) {
    dstp[i] = (float)(radialCount[i] > 0 ? radialSum[i] / radialCount[i] : 0.0f);
  }

  return true;
}

void fftRadialProfileToImage(const cv::Mat1f & radialProfile, const cv::Size & outputImageSize,
    cv::Mat1f & outputImage)
{
  const cv::Size & size = outputImageSize;

  const float cx = size.width / 2;
  const float cy = size.height / 2;
  const int numBins = radialProfile.cols;

  const float scaleX = float(1.f / cx);
  const float scaleY = float(1.f / cy);
  const float binScale = float(numBins * M_SQRT1_2);

  outputImage.create(size);

  parallel_for(0, size.height, [=, &radialProfile, &outputImage](const auto & range) {

    const float * bins = radialProfile[0];

    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = outputImage[y];

      const float dy = (y - cy) * scaleY;
      const float dy2 = dy * dy;

      for (int x = 0; x < size.width; ++x) {
        const float dx = (x - cx) * scaleX;
        const float dx2 = dx * dx;
        const float r = std::sqrt(dx2 + dy2);
        const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);
        dstp[x] = bins[bin];
      }
    }
  });
}

// DFT Radial Profile for packed OpenCV CCS spectrum.
// Include corners. The max dimensionless radius at the corner of the frame is sqrt(1^2 + 1^2) = sqrt(2)
bool fftRadialProfileCCS(const cv::Mat1f & ccsSpectrum, cv::Mat1f & outputProfile)
{
  INSTRUMENT_REGION("");

  if( ccsSpectrum.empty() ) {
    return false;
  }

  const int rows = ccsSpectrum.rows;
  const int cols = ccsSpectrum.cols;
  const float cx = cols / 2.0f;
  const float cy = rows / 2.0f;
  const float R = std::sqrt(cx * cx + cy * cy);
  const int numBins = std::max(1, cvRound(R));

  std::vector<float> globalSum(numBins, 0.0f);
  std::vector<float> globalCount(numBins, 0.0f);
  std::mutex mtx;

  const bool is_even_x = (cols % 2 == 0);
  const bool is_even_y = (rows % 2 == 0);
  const float scaleX = float(1.0 / cx);
  const float scaleY = float(1.0 / cy);
  const float binScale = float(numBins * M_SQRT1_2);

  const uint8_t * spec_base = ccsSpectrum.ptr();
  const size_t spec_stride = ccsSpectrum.step;

  parallel_for(0, rows, [=, &globalSum, &globalCount, &mtx](const auto & range) {

    std::vector<float> localSum(numBins, 0.0f);
    std::vector<float> localCount(numBins, 0.0f);

    float * __restrict lsump = localSum.data();
    float * __restrict lcountp = localCount.data();

    for( int y = rbegin(range); y < rend(range); ++y ) {
      const float * srcp = (const float * )(spec_base + y * spec_stride);

      // True vertical frequency for outer axes (Intel IPL logic)
      int true_freq_y = 0;
      bool is_pure_real_axis_point = false;
      if (y == 0) {
        true_freq_y = 0; // DC
        is_pure_real_axis_point = true;
      }
      else if (is_even_y && y == rows - 1) {
        true_freq_y = rows / 2; // Vertical Nyquist
        is_pure_real_axis_point = true;
      }
      else { // Even rows of the CCS matrix contain Im components; odd rows contain Re components.
        true_freq_y = ((y & 1) == 0) ? (y / 2) : ((y + 1) / 2);
      }

      // Dimensionless dy for vertical axes
      const float dy = true_freq_y * scaleY;
      const float dy2 = dy * dy;

      // DC(X = 0) Vertical axis
      if ( true ) {
        const float r = std::sqrt(dy2);
        const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);

        if (is_pure_real_axis_point) {
          // DC or vertical Nyquist — unique isolated real spectral points
          lsump[bin] += std::abs(srcp[0]);
          lcountp[bin] += 1.0f;
        }
        else {
          // Intermediate axis cell (Re or Im). Since the x=0 axis is symmetric
          // only with respect to the left/right half-plane, its weight is 2.
          // Each component (Re and Im) will be added independently during the parallel_for pass.
          lsump[bin] += std::abs(srcp[0]) * 2.0f;
          lcountp[bin] += 2.0f;
        }
      }

      // For internal columns (X = 1 ... half_cols - 1), the physical frequency along Y follows
      // the standard grid of the unshifted spectrum.
      // Each point is complex and has a hidden mirror counterpart in the right (unpacked) part of the spectrum.
      // Therefore, its weight is strictly equal to 2.
      const float dy_inner = float((y <= rows / 2) ? y : (rows - y)) * scaleY;
      const float dy2_inner = dy_inner * dy_inner;
      const int half_cols = (cols + 1) / 2;
      for (int x = 1; x < half_cols; ++x) {
        const float dx = x * scaleX;
        const float r = std::sqrt(dx * dx + dy2_inner);
        const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);
        const float re = srcp[2 * x - 1];
        const float im = srcp[2 * x];
        const float mag = std::sqrt(re * re + im * im);
        lsump[bin] += mag * 2;
        lcountp[bin] += 2;
      }

      // Nyquist column X = N/2 if cols is even, the physical index is cols - 1
      if (is_even_x) {
        const int last_ccs_col = cols - 1;
        const float dx = (cols / 2) * scaleX;
        const float dx2 = dx * dx;
        const float r = std::sqrt(dx2 + dy2);
        const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);

        if (is_pure_real_axis_point) {
          // Horizontal (0, N/2) or diagonal (M/2, N/2) Nyquist — purely real points
          lsump[bin] += std::abs(srcp[last_ccs_col]);
          lcountp[bin] += 1.0f;
        }
        else {
          // Intermediate horizontal Nyquist component (broadcast to left/right, weight 2)
          lsump[bin] += std::abs(srcp[last_ccs_col]) * 2.0f;
          lcountp[bin] += 2.0f;
        }
      }
    }

    std::lock_guard<std::mutex> lock(mtx);
    for (int b = 0; b < numBins; ++b) {
      globalSum[b] += lsump[b];
      globalCount[b] += lcountp[b];
    }
  });

  outputProfile.create(1, numBins);
  float * __restrict dstp = outputProfile.ptr<float>(0);
  for( int i = 0; i < numBins; ++i ) {
    dstp[i] = (globalCount[i] > 0.0f) ? (globalSum[i] / globalCount[i]) : 0.0f;
  }

  return true;
}



bool dctRadialProfile(const cv::Mat1f & dctSpectrum, cv::Mat1f & outputProfile)
{
  INSTRUMENT_REGION("");
  if( dctSpectrum.empty() ) {
    return false;
  }

  const int maxW = dctSpectrum.cols;
  const int maxH = dctSpectrum.rows;

  const float R = std::sqrt(maxW * maxW + maxH * maxH);
  const int numBins = std::max(1, int(R));
  const float binScale = float(numBins * M_SQRT1_2);

  std::vector<float> radialSum(numBins, 0.0f);
  std::vector<float> radialCount(numBins, 0.0f);

  const float scaleX = float(1.0 / maxW);
  const float scaleY = float(1.0 / maxH);

  for( int y = 0; y < maxH; ++y ) {
    const float dy = y * scaleY;
    const float dy2 = dy * dy;
    const float * srcp = dctSpectrum[y];

    for( int x = 0; x < maxW; ++x ) {
      const float dx = x * scaleX;
      const float dx2 = dx * dx;
      const float r = std::sqrt(dx2 + dy2);
      const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);
      radialSum[bin] += std::abs(srcp[x]);
      radialCount[bin] += 1;
    }
  }

  outputProfile.create(1, numBins);
  float * __restrict dstp = outputProfile[0];
  for( int i = 0; i < numBins; ++i ) {
    dstp[i] = (float) (radialCount[i] > 0 ? radialSum[i] / radialCount[i] : 0.0f);
  }

  return true;
}

void dctRadialProfileToImage(const cv::Mat1f & radialProfile, const cv::Size & outputImageSize,
    cv::Mat1f & outputImage)
{
  const cv::Size & size = outputImageSize;

  const int numBins = radialProfile.cols;
  const float binScale = float(numBins * M_SQRT1_2);

  const float scaleX = float(1. / size.width);
  const float scaleY = float(1. / size.height);

  outputImage.create(size);

  parallel_for(0, size.height, [=, &radialProfile, &outputImage](const auto & range) {

    const float * bins = radialProfile[0];

    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = outputImage[y];

      const float dy = y * scaleY;
      const float dy2 = dy * dy;

      for (int x = 0; x < size.width; ++x) {
        const float dx = x * scaleX;
        const float dx2 = dx * dx;
        const float r = std::sqrt(dx2 + dy2);
        const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);
        dstp[x] = bins[bin];
      }
    }
  });
}

bool fftAccumulatePowerSpectrum(const cv::Mat & src,  cv::Mat & acc, float & cnt)
{
  static const auto compute_magnitue =
      [](cv::Mat & src) {

        typedef std::complex<float> complex;

        const cv::Mat_<complex> spec = src;
        cv::Mat1f mag(spec.size());

        for ( int y = 0; y < spec.rows; ++y ) {
          for ( int x = 0; x < spec.cols; ++x ) {
            mag[y][x] = std::abs(spec[y][x]);
          }
        }

        src = std::move(mag);
      };


  cv::Mat img;
  cv::Size fft_size;

  const int cn = src.channels();
  cv::Mat channels[cn];

  if ( !acc.empty() && cnt > 0 ) {
    fft_size = acc.size();
  }
  else {
    fft_size = fftGetOptimalSize(src.size(), cv::Size(32,32));
    acc.create(fft_size, CV_MAKETYPE(CV_32F, src.channels()));
    acc.setTo(0);
    cnt = 0;
  }

  fftCopyMakeBorder(src, img, fft_size, nullptr);

  if ( cn == 1 ) {
    channels[0] = img;
  }
  else {
    cv::split(img, channels);
  }

  for ( int i = 0; i < cn; ++i ) {
    cv::dft(channels[i], channels[i], cv::DFT_COMPLEX_OUTPUT);
    compute_magnitue(channels[i]);
  }

  if ( cn > 1 ) {
    cv::merge(channels, cn, img);
  }
  else {
    img = channels[0];
  }

  cv::add(acc, img, acc);
  ++cnt;

  return true;
}

bool fftMaxPowerSpectrum(const cv::Mat & src, cv::Mat & acc)
{
  static const auto compute_magnitue =
      [](cv::Mat & src) {

        typedef std::complex<float> complex;

        const cv::Mat_<complex> spec = src;
        cv::Mat1f mag(spec.size());

        for ( int y = 0; y < spec.rows; ++y ) {
          for ( int x = 0; x < spec.cols; ++x ) {
            mag[y][x] = std::abs(spec[y][x]);
          }
        }

        src = std::move(mag);
      };


  cv::Size fft_size;
  cv::Mat img, spec;

  const int cn = src.channels();
  cv::Mat channels[cn];

  if ( !acc.empty() ) {
    fft_size = acc.size();
  }
  else {
    fft_size = fftGetOptimalSize(src.size(), cv::Size(32, 32));
    acc.create(fft_size, CV_MAKETYPE(CV_32F, src.channels()));
    acc.setTo(0);
  }

  if ( !fftCopyMakeBorder(src, img, fft_size, nullptr) ) {
    CF_ERROR("copyMakeFFTBorder(src=%dx%d, img, fft_size=%dx%d) fails",
        src.cols, src.rows, fft_size.width, fft_size.height);
    return false;
  }

  if ( cn == 1 ) {
    channels[0] = img;
  }
  else {
    cv::split(img, channels);
  }


  for ( int i = 0; i < cn; ++i ) {
    cv::dft(channels[i], channels[i], cv::DFT_COMPLEX_OUTPUT);
    compute_magnitue(channels[i]);
  }

  if ( cn > 1 ) {
    cv::merge(channels, cn, img);
  }
  else {
    img = channels[0];
  }

  cv::max(acc, img, acc);

  return true;
}

// Space Isotropic Gaussian
cv::Mat1f fftGenerateGaussianFilter(const cv::Size & fftSize, double sigma_space, double gain, bool centerDC)
{
  cv::Mat1f FILTER(fftSize);

  const bool inverseFilter = sigma_space < 0;
  if( inverseFilter ) {
    sigma_space = -sigma_space;
  }
  else if( sigma_space == 0 ) {
    sigma_space = 1;
  }

  const float cx = float(fftSize.width / 2.0);
  const float cy = float(fftSize.height / 2.0);
  const float sx = float(sigma_space * CV_PI * M_SQRT2 / fftSize.width);
  const float sy = float(sigma_space * CV_PI * M_SQRT2 / fftSize.height);
  const float fgain = float(gain);

  parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = FILTER[y];
      const float dy = (y - cy) * sy;
      const float dy2 = dy * dy;
      for (int x = 0; x < fftSize.width; ++x) {
        const float dx = (x - cx) * sx;
        const float dx2 = dx * dx;
        const float gaussLPF = fgain * std::exp(-(dx2 + dy2));
        dstp[x] = float(inverseFilter ? fgain - gaussLPF : gaussLPF);
      }
    }
  });

  if( !centerDC ) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

cv::Mat1f fftGenerateLaplacianFilter(const cv::Size & fftSize,
    double gain /* =1*/,
    bool centerDC /* =true */)
{
  // Isotropic Laplacian
  // The frequency step is tied to the physical dimensions of the matrix
  //  fx = dx / width
  //  fy = dy / height
  // Physical Laplacian:
  //    4 * PI^2 * (fx^2 + fy^2)

  cv::Mat1f FILTER(fftSize);

  const float scaleX = float (CV_2PI / fftSize.width);
  const float scaleY = float (CV_2PI / fftSize.height);
  const float cx = float (fftSize.width / 2.0);
  const float cy = float (fftSize.height / 2.0);
  const float fgain = float (gain);

  parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = FILTER[y];
      const float  dy = (y - cy) * scaleY;
      const float  dy2 = dy * dy;
      for (int x = 0; x < fftSize.width; ++x) {
        const float dx = (x - cx) * scaleX;
        const float dx2 = dx * dx;
        const float dr2 = dx2 + dy2;
        dstp[x] = fgain * dr2;
      }
    }
  });

  if( !centerDC ) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

cv::Mat1f fftGenerateLaplacianUnsharpFilter(const cv::Size & fftSize, double gain, double bwrc, int bworder,
    bool centerDC)
{
  // Isotropic Laplacian
  // The frequency step is tied to the physical dimensions of the matrix
  //    fx = dx / width
  //    fy = dy / height
  // Physical Laplacian:
  //    4 * PI^2 * (fx^2 + fy^2)
  // Isotropic Butterworth:
  //    1.0 / (1.0 + (r / rc)^(n))

  cv::Mat1f FILTER(fftSize);

  parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {

    const float bworder2 = float (bworder / 2.);
    const float bwrc2 = float (1. / (bwrc * bwrc));
    const float fgain = (float) (gain);

    const float scaleX = float (CV_2PI / fftSize.width);
    const float scaleY = float (CV_2PI / fftSize.height);
    const float cx = float (fftSize.width / 2.0);
    const float cy = float (fftSize.height / 2.0);

    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = FILTER[y];
      const float dy = (y - cy) * scaleY;
      const float dy2 = dy * dy;
      for (int x = 0; x < fftSize.width; ++x) {
        const float dx = (x - cx) * scaleX;
        const float dx2 = dx * dx;
        const float dr2 = dx2 + dy2;
        const float v = 1.f + fgain * dr2 / (1.f + std::pow(dr2 * bwrc2, bworder2));
        dstp[x] = v;
      }
    }
  });

  if( !centerDC ) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

cv::Mat1f fftGenerateRampFilter(const cv::Size & fftSize, double gain, bool centerDC)
{
  // Isotropic Gradient
  // The frequency step is tied to the physical dimensions of the matrix
  //    fx = dx / width
  //    fy = dy / height
  // Gradient:
  //    2 * PI * sqrt(fx^2 + fy^2)

  cv::Mat1f FILTER(fftSize);

  const float scaleX = float(CV_2PI / fftSize.width);
  const float scaleY = float(CV_2PI / fftSize.height);

  if( centerDC ) {
    const float cx = float(fftSize.width / 2.0);
    const float cy = float(fftSize.height / 2.0);
    const float fgain = float(gain);

    parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        float * __restrict dstp = FILTER[y];
        const float dy = (y - cy) * scaleY;
        const float dy2 = dy * dy;
        for (int x = 0; x < fftSize.width; ++x) {
          const float dx = (x - cx) * scaleX;
          const float dx2 = dx * dx;
          const float dr = std::sqrt(dx2 + dy2);
          dstp[x] = fgain * dr;
        }
      }
    });
  }
  else {  // !centerDC

    const int cx = fftSize.width / 2;
    const int cy = fftSize.height / 2;

    parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        float * __restrict dstp = FILTER[y];

        const float dy_val = float((y <= cy) ? y : fftSize.height - y);
        const float dy = dy_val * scaleY;
        const float dy2 = dy * dy;

        for (int x = 0; x < fftSize.width; ++x) {
          const float dx_val = float((x <= cx) ? x : fftSize.width - x);
          const float dx = dx_val * scaleX;
          dstp[x] = std::sqrt(dx * dx + dy2);
        }
      }
    });

    FILTER(0, 0) = 0.0f;
  }

  return FILTER;
}


cv::Mat1f dctGenerateRampFilter(const cv::Size & dctSize, double gain)
{
  // Isotropic Gradient for DCT
  // For DCT, the DC component (zero frequency) is strictly at top-left (0,0).
  // Frequencies increase radially towards the bottom-right.
  //  fx = x / width
  //  fy = y / height
  // Gradient:
  //  2 * PI * sqrt(fx^2 + fy^2)

  cv::Mat1f FILTER(dctSize);

  const float scaleX = float(CV_2PI / dctSize.width);
  const float scaleY = float(CV_2PI / dctSize.height);
  const float fgain = float(gain);

  parallel_for(0, dctSize.height, [=, &FILTER](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = FILTER[y];

      const float dy = float(y) * scaleY;
      const float dy2 = dy * dy;

      for (int x = 0; x < dctSize.width; ++x) {
        const float dx = float(x) * scaleX;
        const float dx2 = dx * dx;

        const float dr = std::sqrt(dx2 + dy2);
        dstp[x] = fgain * dr;
      }
    }
  });

  FILTER(0, 0) = 0.0f;

  return FILTER;
}

// Multiplicative Discrete Laplacian Filter for Periodic+Smooth Decomposition
cv::Mat1f fftGenerateDiscreteLaplacianFilter(const cv::Size & fftSize, bool centerDC)
{
  cv::Mat1f FILTER(fftSize);

  const double scaleX = CV_2PI / fftSize.width;
  const double scaleY = CV_2PI / fftSize.height;
  const int cx = centerDC ? fftSize.width / 2 : 0;
  const int cy = centerDC ? fftSize.height / 2 : 0;

  std::vector<double> cosX(fftSize.width);
  std::vector<double> cosY(fftSize.height);

  for (int x = 0; x < fftSize.width; ++x) {
    cosX[x] = std::cos((x - cx) * scaleX);
  }

  for (int y = 0; y < fftSize.height; ++y) {
    cosY[y] = std::cos((y - cy) * scaleY);
  }

  parallel_for(0, fftSize.height,
      [=, cosx = cosX.data(), cosy = cosY.data(), &FILTER](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        float * __restrict dstp = FILTER[y];
        const double cos_y = cosy[y];
        for (int x = 0; x < fftSize.width; ++x) {
          const double denom = 2.0 * (2.0 - cosx[x] - cos_y);
          dstp[x] = float(1.0 / denom);
        }
      }
    });

  FILTER(cy, cx) = 0.f;

  return FILTER;
}

// Isotropic Butterworth: 1.0 / (1.0 + (r / rc)^(n))
cv::Mat1f fftGenerateButterworthFilter(const cv::Size & fftSize,
    double rc, int order, double gain,
    bool centerDC)
{
  // The frequency step is tied to the physical dimensions of the matrix
  // fx = dx / width, fy = dy / height

  cv::Mat1f FILTER(fftSize);

  const double cx = fftSize.width / 2.0;
  const double cy = fftSize.height / 2.0;
  const double sx = CV_2PI / fftSize.width;
  const double sy = CV_2PI / fftSize.height;

  parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float* __restrict dstp = FILTER[y];

      const double dy = (y - cy) * sy;
      const double dy2 = dy * dy;

      for (int x = 0; x < fftSize.width; ++x) {
        const double dx = (x - cx) * sx;
        const double dx2 = dx * dx;

        const double r = std::sqrt(dx2 + dy2);

        dstp[x] = float(gain / (1.0 + std::pow(r / rc, order)));
      }
    }
  });

  if( !centerDC ) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

// Space Isotropic Gaussian-Based unsharp mask filter
cv::Mat1f fftGenerateGaussianUnsharpFilter(const cv::Size & fftSize,
    double sigma_space, double gain, bool centerDC)
{
  cv::Mat1f FILTER(fftSize);

  const bool inverseFilter = sigma_space < 0;
  if (inverseFilter) {
    sigma_space = -sigma_space;
  }
  else if (sigma_space == 0) {
    sigma_space = 1;
  }

  const double cx = fftSize.width / 2.0;
  const double cy = fftSize.height / 2.0;
  const double sx = sigma_space * CV_PI * M_SQRT2 / fftSize.width;
  const double sy = sigma_space * CV_PI * M_SQRT2 / fftSize.height;

  parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict dstp = FILTER[y];

      const double dy = (y - cy) * sy;
      const double dy2 = dy * dy;

      for (int x = 0; x < fftSize.width; ++x) {
        const double dx = (x - cx) * sx;
        const double dx2 = dx * dx;

        const double gaussLPF = std::exp(-(dx2 + dy2));
        const double unsharpHPF = 1.0 + gain * (1.0 - gaussLPF);

        // Unsharp Mask: 1.0 + alpha * (1.0 - LPF)
        dstp[x] = float(inverseFilter ? (1.0 + gain) - unsharpHPF : unsharpHPF);
      }
    }
  });

  if( !centerDC ) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

// Space Isotropic Butterworth-Based unsharp mask filter
cv::Mat1f fftGenerateButterworthUnsharpFilter(const cv::Size & fftSize,
    double rc_space, double order, double gain, bool centerDC)
{
  cv::Mat1f FILTER(fftSize);

  if (rc_space <= 0.0) {
    rc_space = 1.0;
  }

  const double rc = CV_2PI / rc_space;
  const double cx = fftSize.width / 2.0;
  const double cy = fftSize.height / 2.0;
  const double sx = CV_2PI / fftSize.width;
  const double sy = CV_2PI / fftSize.height;

  parallel_for(0, fftSize.height, [=, &FILTER](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float* __restrict dstp = FILTER[y];

      const double dy = (y - cy) * sy;
      const double dy2 = dy * dy;

      for (int x = 0; x < fftSize.width; ++x) {
        const double dx = (x - cx) * sx;
        const double dx2 = dx * dx;

        const double r2 = dx2 + dy2;
        double butterworthLPF = 1.0;

        if (r2 > 0.0) {
          const double r = std::sqrt(r2);
          butterworthLPF = 1.0 / (1.0 + std::pow(r / rc, order));
        }

        // Unsharp Mask: 1.0 + alpha * (1.0 - LPF)
        dstp[x] = float(1.0 + gain * (1.0 - butterworthLPF));
      }
    }
  });

  if (!centerDC) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

/**
 * @brief Generates an inverse frequency-domain cross-shaped filter to suppress rectangular window artifacts.
 *
 * This function constructs a 2D weighting matrix in the frequency domain designed to mitigate
 * spectral leakage ("cross-shaped" or "ray" artifacts) caused by the rectangular boundaries
 * of the input ROI (Region of Interest) during FFT-based cross- or phase correlation.
 * It models the vertical and horizontal leakage profiles and applies Tikhonov regularization
 * to invert the artifact field safely without noise amplification.
 *
 * @param[in]  fftSize   The size of the FFT grid (total dimensions of the spectrum matrix).
 * @param[in]  rectSize  The dimensions of the source rectangular window/ROI causing the leakage.
 * @param[out] dst       Output 2D single-channel matrix (`CV_32FC1`) containing the generated filter.
 * @param[in]  _csigma   Characteristic scale factor controlling the width/decay of the artifact rays.
 * @param[in]  _calpha   Tikhonov regularization parameter. Controls the stability of the inversion
 *                       near the high-amplitude artifact zones (prevents division by zero).
 * @param[in]  centerDC  If true, the DC component (zero frequency) is assumed to be in the center
 *                       of the matrix (shifted FFT). If false, DC is at (0,0).
 *
 * @note Implements multi-threaded generation using `parallel_for` for optimal performance.
 */
void fftGenerateInverseCrossFilter(const cv::Size & fftSize, const cv::Size & rectSize, cv::OutputArray dst,
    double _csigma, double _calpha, bool centerDC)
{
  if( fftSize.width <= 0 || fftSize.height <= 0 || rectSize.width <= 0 || rectSize.height <= 0 ) {
    dst.release();
    return;
  }

  dst.create(fftSize, CV_32FC1);
  cv::Mat1f mask = dst.getMatRef();

  const int cx = centerDC ? fftSize.width / 2 : 0;
  const int cy = centerDC ? fftSize.height / 2 : 0;

  const float sigma2 = float(_csigma * _csigma);
  const float alpha2 = float(_calpha * _calpha);
  const float W = float(rectSize.width);
  const float H = float(rectSize.height);
  const float N = float(fftSize.width);
  const float M = float(fftSize.height);

  const float dutyx = float(N)/float(W);
  const float dutyy = float(M)/float(H);
  const float amp_horz_ray = dutyx * dutyx;
  const float amp_vert_ray = dutyy * dutyy;

  const uint8_t * filter_base = mask.ptr();
  const size_t filter_stride = mask.step;

  parallel_for(0, fftSize.height, [=](const auto & range) {
    for( int y = rbegin(range); y < rend(range); ++y ) {
      float * __restrict fltp = (float * )(filter_base + y * filter_stride);

      int dy = std::abs(y - cy);
      if( !centerDC && dy > fftSize.height / 2 ) {
        dy = fftSize.height - dy;
      }

      // The horizontal ray profile depends on the vertical frequency dy
      // The vertical ray profile depends on the horizontal frequency dx
      const float horz_ray_profile = amp_horz_ray / (1.0f + (float(dy * dy) / sigma2));

      for( int x = 0; x < fftSize.width; ++x ) {
        int dx = std::abs(x - cx);
        if( !centerDC && dx > fftSize.width / 2 ) {
          dx = fftSize.width - dx;
        }

        // Total artifact field with Tikhonov regularization
        const float vert_ray_profile = amp_vert_ray / (1.0f + (float(dx * dx) / sigma2));
        const float cross_amplitude = horz_ray_profile + vert_ray_profile - horz_ray_profile * vert_ray_profile;
        fltp[x] = 1.0f / (1.0f + cross_amplitude * cross_amplitude / alpha2);
      }
    }
  });
}

bool fftMulSpectrum(const cv::Mat1f & filter, cv::InputArray complexSpectrum,
    cv::OutputArray dst)
{
  cv::Mat2f F;
  const cv::Mat planes[] {
      filter, filter
  };
  cv::merge(planes, 2, F);
  cv::multiply(F, complexSpectrum, dst);

  return true;
}


// Create V-Matrix for Periodic+Smooth Decomposition
void fftCreateVMatrix(cv::InputArray _src, cv::OutputArray _dst)
{
  INSTRUMENT_REGION("");

  const cv::Mat src = _src.getMat();
  const int rows = src.rows;
  const int cols = src.cols;

  cv::Mat dst = cv::Mat::zeros(src.size(), CV_MAKETYPE(CV_32F, src.channels()));

  cv::Mat d;
  cv::subtract(src.row(0), src.row(rows - 1), d, cv::noArray(), CV_32F);
  cv::add(dst.row(0), d, dst.row(0), cv::noArray(), CV_32F);
  cv::subtract(dst.row(rows - 1), d, dst.row(rows - 1), cv::noArray(), CV_32F);

  cv::subtract(src.col(0), src.col(cols - 1), d, cv::noArray(), CV_32F);
  cv::add(dst.col(0), d, dst.col(0), cv::noArray(), CV_32F);
  cv::subtract(dst.col(cols - 1), d, dst.col(cols - 1), cv::noArray(), CV_32F);

  _dst.move(dst);
}

// DFT with Periodic + Smooth Decomposition.
// The Inverse Discrete Laplacian Filter VLAP must be prepared before this call.
// const cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, true);
// The target fftSize (FFT padding) is defined by the VLAP.size()
void fftPPSDecomposition(cv::InputArray src_image, const cv::Mat1f & VLAP,
    cv::OutputArray P_SPECTRUM, cv::OutputArray S_SPECTRUM,
    bool centerDC)
{
  cv::Mat SRC, SRC_SPECTRUM, V;

  const cv::Size fftSize = VLAP.size();

  if ( src_image.size() == fftSize ) {
    SRC = src_image.getMat();
  }
  else {
    fftCopyMakeBorder(src_image, SRC, fftSize);
  }

  fftCreateVMatrix(SRC, V);

  cv::dft(SRC, SRC_SPECTRUM, cv::DFT_COMPLEX_OUTPUT);
  cv::dft(V, V, cv::DFT_COMPLEX_OUTPUT);
  if( centerDC ) {
    fftSwapQuadrants(SRC_SPECTRUM);
    fftSwapQuadrants(V);
  }

  if ( S_SPECTRUM.needed() ) {
    fftMulSpectrum(VLAP, V, S_SPECTRUM);
    cv::subtract(SRC_SPECTRUM, S_SPECTRUM, P_SPECTRUM);
  }
  else {
    cv::Mat S_SPECTRUM_TMP;
    fftMulSpectrum(VLAP, V, S_SPECTRUM_TMP);
    cv::subtract(SRC_SPECTRUM, S_SPECTRUM_TMP, P_SPECTRUM);
  }
}

void fftPPSDecomposition(cv::InputArray src_image, const cv::Mat1f & VLAP,
    std::vector<cv::Mat2f> * P_SPECTRUMS, std::vector<cv::Mat2f> * S_SPECTRUMS,
    bool centerDC)
{
  cv::Mat SRC, SRC_SPECTRUM, V;

  const cv::Size fftSize = VLAP.size();
  const int cn = src_image.channels();
  std::vector<cv::Mat> src_channels(cn);

  if ( src_image.size() == fftSize ) {
    SRC = src_image.getMat();
  }
  else {
    fftCopyMakeBorder(src_image, SRC, fftSize);
  }

  if ( cn == 1 ) {
    src_channels[0] = SRC;
  }
  else {
    cv::split(SRC, src_channels);
  }

  P_SPECTRUMS->resize(cn);
  S_SPECTRUMS->resize(cn);

  for ( int i = 0; i < cn; ++i ) {
    fftCreateVMatrix(src_channels[i], V);

    cv::dft(V, V, cv::DFT_COMPLEX_OUTPUT);
    cv::dft(src_channels[i], SRC_SPECTRUM, cv::DFT_COMPLEX_OUTPUT);

    if( centerDC ) {
      fftSwapQuadrants(SRC_SPECTRUM);
      fftSwapQuadrants(V);
    }

    fftMulSpectrum(VLAP, V, S_SPECTRUMS->at(i));
    cv::subtract(SRC_SPECTRUM, S_SPECTRUMS->at(i), P_SPECTRUMS->at(i));
  }
}




/**
* @brief Create smooth circular cosine window to mask the corners of a square.
* for planetary disk ROIs
*/
cv::Mat1f fftCreateCircularApodizationWindow(const cv::Size & size)
{
  cv::Mat1f mask = cv::Mat1f::zeros(size);

  const float r_outer = (size.width / 2.0f) * 0.95f;
  const float r_inner = r_outer * 0.65f;

  parallel_for(0, size.height, [=, &mask](const auto & range) {
    const int cx = size.width / 2;
    const int cy = size.height / 2;

    for( int y = rbegin(range); y < rend(range); ++y ) {
      float * __restrict dstp = mask[y];
      for( int x = 0; x < mask.cols; ++x ) {
        const float dx = x - cx;
        const float dy = y - cy;
        const float r = std::sqrt(dx * dx + dy * dy);

        if( r <= r_inner ) {
          dstp[x] = 1.0f;
        }
        else if( r >= r_outer ) {
          dstp[x] = 0.0f;
        }
        else {
          const float fraction = (r - r_inner) * float(CV_PI) / (r_outer - r_inner);
          dstp[x] = 0.5f * (1.0f + std::cos(fraction));
        }
      }
    }
  });

  return mask;
}


/**
* @brief Function for automatically determining the position angle from the FFT spectrum module
* @param fftSpectrum Cleaned FFT spectrum (after ppsDecomposition and morphological smoothing)
* @return double Polar axis position angle in degrees [0, 180)
*/
double fftEstimateRadonOrientation(const cv::Mat1f & fftSpectrum,
    cv::OutputArray outputDebugHistogram /*= cv::noArray()*/)
{
  const cv::Size fftSize = fftSpectrum.size();
  const int cx = fftSize.width / 2;
  const int cy = fftSize.height / 2;

  // Step over only the very core (15 px) and
  // take the entire beam up to the mid frequencies (150 px)
  const int Rmin = 5;
  const int Rmax = 0.33 * cx;
  const double Rmin2 = Rmin * Rmin;
  const double Rmax2 = Rmax * Rmax;

  // Resolution of Nyquist histogram
  const int num_bins = std::max(180, cvRound(CV_PI * Rmax));
  const double bin_step = 180.0 / num_bins;

  cv::Mat1f HIST(1, num_bins, 0.0f);
  float * __restrict histp = HIST[0];

  // Scan range
  const int y_start = std::max(0, cy - Rmax);
  const int y_end   = std::min(fftSpectrum.rows, cy + Rmax);
  const int x_start = std::max(0, cx - Rmax);
  const int x_end   = std::min(fftSpectrum.cols, cx + Rmax);

  for (int y = y_start; y < y_end; ++y) {
    const float * srcp = fftSpectrum[y];
    const double dy = y - cy;
    const double dy2 = dy * dy;

    for (int x = x_start; x < x_end; ++x) {
      const double dx = x - cx;
      const double dx2 = dx * dx;
      const double r2 = dx2 + dy2;

      if( r2 >= Rmin2 && r2 <= Rmax2 ) {

        // Collapse symmetric FFT spectrum into a hemisphere [0, 180)
        double angle = std::atan2(dy, dx) * 180.0 / CV_PI;
        if( angle < 0 ) {
          angle += 180;
        }

        const int bin_idx = int(angle / bin_step);
        if( bin_idx >= 0 && bin_idx < num_bins ) {
          //const float w = float(srcp[x] * sqrt(r2));
          const float w = float(srcp[x]);
          histp[bin_idx] += w;
        }
      }
    }
  }

  cv::Mat1f H, Hp;
  double globalMin, globalMax, globalRange;
  cv::Point globalMaxPos;

  const int border = num_bins / 2;
  const cv::Rect roi(border, 0, num_bins, 1);
  const int hksize = 55;
  const int hpksize = 25;

  cv::copyMakeBorder(HIST, H, 0, 0, border, border, cv::BORDER_WRAP);
  cv::GaussianBlur(H, H, cv::Size(hksize, 1), 0, 0);
  cv::GaussianBlur(H, Hp, cv::Size(hpksize, 1), 0, 0);
  cv::subtract(H, Hp, Hp);

  if( outputDebugHistogram.needed() ) {
    cv::transpose(H(roi), outputDebugHistogram);
  }

  cv::minMaxLoc(H(roi), &globalMin, &globalMax, nullptr, &globalMaxPos);
  if( !((globalRange = globalMax - globalMin) > 0) ) {
    return 0;
  }

  const int center = globalMaxPos.x + border;
  double X = center * H(0, center);
  double W = H(0, center);

  for( int i = center - 1; i >= 0 && Hp(0, i) > 0; --i ) {
    const double h = H(0, i);
    X += i * h;
    W += h;
  }
  for( int i = center + 1; i < H.cols && Hp(0, i) > 0; ++i ) {
    const double h = H(0, i);
    X += i * h;
    W += h;
  }

  const double angle = (X / W - border) * bin_step;
  //CF_DEBUG("FFT: angle=%g", angle);

  return angle;
}

/**
 * CV_32FC1 CCS input -> CV_32FC2 Complex output
 * */
bool fftUnpackCCSSpectrum(cv::InputArray _ccsSpectrum, cv::OutputArray _complexSpectrum)
{
  if ( _ccsSpectrum.empty() || _ccsSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("CV_32FC1 CCS packed input spectrum expected");
    return false;
  }
  if ( _complexSpectrum.fixedType() && _complexSpectrum.type() != CV_32FC2 ) {
    CF_ERROR("CV_32FC2 unpacked complex output spectrum destination expected");
    return false;
  }

  const int rows = _ccsSpectrum.rows(); // M
  const int cols = _ccsSpectrum.cols(); // N
  const int half_rows = (rows + 1) / 2;
  const int half_cols = (cols + 1) / 2;
  const bool has_even_rows = (rows & 1) == 0;
  const bool has_even_cols = (cols & 1) == 0;

  const cv::Mat1f ccsSpectrum = _ccsSpectrum.getMat();
  const uint8_t * ccs_base = ccsSpectrum.ptr();
  const size_t ccs_stride = ccsSpectrum.step;

  _complexSpectrum.create(rows, cols, CV_32FC2);
  cv::Mat2f complexSpectrum = _complexSpectrum.getMatRef();
  uint8_t * cmplx_base = complexSpectrum.ptr();
  const size_t cmplx_stride = complexSpectrum.step;

  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      cv::Vec2f * __restrict cmplxp = (cv::Vec2f*)(cmplx_base + y * cmplx_stride);

      // DC (X = 0) Packed vertically by IPL table rows
      if (y == 0) {
        const float * ccs_re = (const float *)(ccs_base + 0 * ccs_stride);
        cmplxp[0][0] = ccs_re[0]; // ReY_0,0 (DC)
        cmplxp[0][1] = 0.0f;    // ImY_0,0 always 0
      }
      else if (has_even_rows && y == rows / 2) {
        const float * ccs_re = (const float *)(ccs_base + (rows - 1) * ccs_stride);
        cmplxp[0][0] = ccs_re[0]; // ReY_M/2,0 Vertical Nyquist
        cmplxp[0][1] = 0.0f;            // ImY_M/2,0 always 0
      }
      else if (y < half_rows) {
        const float * ccs_re = (const float *)(ccs_base + (2 * y - 1) * ccs_stride);
        const float * ccs_im = (const float *)(ccs_base + (2 * y) * ccs_stride);
        cmplxp[0][0] = ccs_re[0]; // ReY_y,0
        cmplxp[0][1] = ccs_im[0]; // ImY_y,0
      }
      else { // y > rows / 2 Lower conjugation half-plane
        const int sym_y = rows - y;
        const float * ccs_re = (const float *)(ccs_base + (2 * sym_y - 1) * ccs_stride);
        const float * ccs_im = (const float *)(ccs_base + (2 * sym_y) * ccs_stride);
        cmplxp[0][0] = ccs_re[0];  // ReY_y,0 = ReY_sym_y,0
        cmplxp[0][1] = -ccs_im[0]; // ImY_y,0 = -ImY_sym_y,0
      }

      // Inner columns iterate up to (cols + 1) / 2 exclusive,
      // so the Nyquist frequency (cols / 2) is not included when cols is even.
      // The left half from the current CCS row, right half from symmetric CCS row
      const float * ccsp = (const float * )(ccs_base + y * ccs_stride);
      const float * ccsp_sym = (const float *)(ccs_base + (y ? (rows - y) : 0 ) * ccs_stride);
      for (int x = 1; x < half_cols; ++x) {
        const int sym_x = cols - x;
        cmplxp[x][0] = ccsp[2 * x - 1]; // Re
        cmplxp[x][1] = ccsp[2 * x];     // Im
        cmplxp[sym_x][0] = ccsp_sym[2 * x - 1];  // Re the same as symmetric element
        cmplxp[sym_x][1] = -ccsp_sym[2 * x];     // Im conjugation
      }

      // Nyquist column X = N/2 packed vertically in the last column N-1
      if (has_even_cols) {
        const int last_ccs_col = cols - 1;
        if (y == 0) {
          const float * ccs_re = (const float *)(ccs_base + 0 * ccs_stride);
          cmplxp[cols / 2][0] = ccs_re[last_ccs_col]; // ReY_0,N/2
          cmplxp[cols / 2][1] = 0.0f;
        }
        else if (has_even_rows && y == rows / 2) {
          const float * ccs_re = (const float *)(ccs_base + (rows - 1) * ccs_stride);
          cmplxp[cols / 2][0] = ccs_re[last_ccs_col]; // ReY_M/2,N/2
          cmplxp[cols / 2][1] = 0.0f;
        }
        else if (y < half_rows) {
          const float * ccs_re = (const float *)(ccs_base + (2 * y - 1) * ccs_stride);
          const float * ccs_im = (const float *)(ccs_base + (2 * y) * ccs_stride);
          cmplxp[cols / 2][0] = ccs_re[last_ccs_col]; // ReY_y,N/2
          cmplxp[cols / 2][1] = ccs_im[last_ccs_col]; // ImY_y,N/2
        }
        else {
          const int sym_y = rows - y;
          const float * ccs_re = (const float *)(ccs_base + (2 * sym_y - 1) * ccs_stride);
          const float * ccs_im = (const float *)(ccs_base + (2 * sym_y) * ccs_stride);
          cmplxp[cols / 2][0] = ccs_re[last_ccs_col];  // ReY_y,N/2
          cmplxp[cols / 2][1] = -ccs_im[last_ccs_col]; // ImY_y,N/2 complex conjugation
        }
      }

    }
  });

  return true;
}

/**
 * CV_32FC1 CCS input -> CV_32FC2 Complex output with sign alternating (avoid later fftShift)
 * */
bool fftUnpackCCSSpectrumAlternateSign(cv::InputArray _ccsSpectrum, cv::OutputArray _complexSpectrum)
{
  if( _ccsSpectrum.empty() || _ccsSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("CV_32FC1 CCS packed input spectrum expected");
    return false;
  }
  if( _complexSpectrum.fixedType() && _complexSpectrum.type() != CV_32FC2 ) {
    CF_ERROR("CV_32FC2 unpacked complex output spectrum destination expected");
    return false;
  }

  const int rows = _ccsSpectrum.rows(); // M
  const int cols = _ccsSpectrum.cols(); // N
  const int half_rows = (rows + 1) / 2;
  const int half_cols = (cols + 1) / 2;
  const bool has_even_rows = (rows & 1) == 0;
  const bool has_even_cols = (cols & 1) == 0;

  const cv::Mat1f ccsSpectrum = _ccsSpectrum.getMat();
  const uint8_t * ccs_base = ccsSpectrum.ptr();
  const size_t ccs_stride = ccsSpectrum.step;

  _complexSpectrum.create(rows, cols, CV_32FC2);
  cv::Mat2f complexSpectrum = _complexSpectrum.getMatRef();
  uint8_t * cmplx_base = complexSpectrum.ptr();
  const size_t cmplx_stride = complexSpectrum.step;

  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      cv::Vec2f * __restrict cmplxp = (cv::Vec2f*)(cmplx_base + y * cmplx_stride);

      // Sign for current row at position X = 0. Changes at each y-step.
      const int row_sign = (y & 1) ? -1 : 1;

      // DC (X = 0) Packed vertically by IPL table rows
      if (y == 0) {
        const float * ccs_re = (const float *)(ccs_base + 0 * ccs_stride);
        cmplxp[0][0] = ccs_re[0] * row_sign; // ReY_0,0 (DC)
        cmplxp[0][1] = 0.0f;
      }
      else if (has_even_rows && y == rows / 2) {
        const float * ccs_re = (const float *)(ccs_base + (rows - 1) * ccs_stride);
        cmplxp[0][0] = ccs_re[0] * row_sign; // ReY_M/2,0 Vertical Nyquist
        cmplxp[0][1] = 0.0f;
      }
      else if (y < half_rows) {
        const float * ccs_re = (const float *)(ccs_base + (2 * y - 1) * ccs_stride);
        const float * ccs_im = (const float *)(ccs_base + (2 * y) * ccs_stride);
        cmplxp[0][0] = ccs_re[0] * row_sign; // ReY_y,0
        cmplxp[0][1] = ccs_im[0] * row_sign;// ImY_y,0
      }
      else { // y > rows / 2 Lower conjugation half-plane
        const int sym_y = rows - y;
        const float * ccs_re = (const float *)(ccs_base + (2 * sym_y - 1) * ccs_stride);
        const float * ccs_im = (const float *)(ccs_base + (2 * sym_y) * ccs_stride);
        cmplxp[0][0] = ccs_re[0] * row_sign;// ReY_y,0
        cmplxp[0][1] = -ccs_im[0] * row_sign;// ImY_y,0
      }

      // Inner columns iterate up to (cols + 1) / 2 exclusive,
      // so the Nyquist frequency (cols / 2) is not included when cols is even.
      // The left half from the current CCS row, right half from symmetric CCS row
      const float * ccsp = (const float * )(ccs_base + y * ccs_stride);
      const float * ccsp_sym = (const float *)(ccs_base + (y ? (rows - y) : 0 ) * ccs_stride);

      for (int x = 1; x < half_cols; ++x) {
        // The sign alternating along the x-axis depending on the parity of the coordinate.
        const int sym_x = cols - x;
        const int sign_x = (x & 1) ? -row_sign : row_sign;
        const int sign_sym_x = (sym_x & 1) ? -row_sign : row_sign;

        cmplxp[x][0] = ccsp[2 * x - 1] * sign_x;// Re Left
        cmplxp[x][1] = ccsp[2 * x] * sign_x;// Im Left
        cmplxp[sym_x][0] = ccsp_sym[2 * x - 1] * sign_sym_x;// Re Right
        cmplxp[sym_x][1] = -ccsp_sym[2 * x] * sign_sym_x;// Im Right (conjugation)
      }

      // =====================================================================
      // Nyquist column X = N/2 packed vertically in the last column N-1
      // =====================================================================
      if (has_even_cols) {
        const int x = cols / 2;
        const int last_ccs_col = cols - 1;
        const int sign_x = (x & 1) ? -row_sign : row_sign;

        if (y == 0) {
          const float * ccs_re = (const float *)(ccs_base + 0 * ccs_stride);
          cmplxp[x][0] = ccs_re[last_ccs_col] * sign_x; // ReY_0,N/2
          cmplxp[x][1] = 0.0f;
        }
        else if (has_even_rows && y == rows / 2) {
          const float * ccs_re = (const float *)(ccs_base + (rows - 1) * ccs_stride);
          cmplxp[x][0] = ccs_re[last_ccs_col] * sign_x; // ReY_M/2,N/2
          cmplxp[x][1] = 0.0f;
        }
        else if (y < half_rows) {
          const float * ccs_re = (const float *)(ccs_base + (2 * y - 1) * ccs_stride);
          const float * ccs_im = (const float *)(ccs_base + (2 * y) * ccs_stride);
          cmplxp[x][0] = ccs_re[last_ccs_col] * sign_x; // ReY_y,N/2
          cmplxp[x][1] = ccs_im[last_ccs_col] * sign_x;// ImY_y,N/2
        }
        else {
          const int sym_y = rows - y;
          const float * ccs_re = (const float *)(ccs_base + (2 * sym_y - 1) * ccs_stride);
          const float * ccs_im = (const float *)(ccs_base + (2 * sym_y) * ccs_stride);
          cmplxp[x][0] = ccs_re[last_ccs_col] * sign_x;  // ReY_y,N/2
          cmplxp[x][1] = -ccs_im[last_ccs_col] * sign_x;// ImY_y,N/2 complex conjugation
        }
      }
    }
  });

  return true;
}

/**
 * CV_32FC2 Complex input -> CV_32FC1 CCS packed output
 * Pack full complex spectrum of the signal into OpenCV CCS format.
 **/
bool fftPackCCSSpectrum(cv::InputArray _complexSpectrum, cv::OutputArray _ccsSpectrum)
{
  if ( _complexSpectrum.empty() || _complexSpectrum.type() != CV_32FC2 ) {
    CF_ERROR("CV_32FC2 complex input spectrum expected");
    return false;
  }
  if ( _ccsSpectrum.fixedType() && _ccsSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("CV_32FC1 packed CCS output spectrum destination expected");
    return false;
  }

  const int rows = _complexSpectrum.rows(); // M
  const int cols = _complexSpectrum.cols(); // N
  const int half_rows = (rows + 1) / 2;
  const int half_cols = (cols + 1) / 2;
  const bool has_even_rows = (rows & 1) == 0;
  const bool has_even_cols = (cols & 1) == 0;

  const cv::Mat2f complexSpectrum = _complexSpectrum.getMat();
  const uint8_t * cmplx_base = complexSpectrum.ptr();
  const size_t cmplx_stride = complexSpectrum.step;

  _ccsSpectrum.create(rows, cols, CV_32FC1);
  cv::Mat1f ccsSpectrum = _ccsSpectrum.getMatRef();
  uint8_t * ccs_base = ccsSpectrum.ptr();
  const size_t ccs_stride = ccsSpectrum.step;

  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      const cv::Vec2f * cmplxp = (const cv::Vec2f*)(cmplx_base + y * cmplx_stride);

      // DC (X = 0) Packed vertically by IPL table rows
      if (y == 0) {
        float * ccs_re = (float *)(ccs_base + 0 * ccs_stride);
        ccs_re[0] = cmplxp[0][0]; // ReY_0,0 (DC is always real)
      }
      else if (has_even_rows && y == rows / 2) {
        float * ccs_re = (float *)(ccs_base + (rows - 1) * ccs_stride);
        ccs_re[0] = cmplxp[0][0]; // ReY_M/2.0 (Vertical Nyquist is also real-valued)
      }
      else if (y < half_rows) {
        float * ccs_re = (float *)(ccs_base + (2 * y - 1) * ccs_stride);
        float * ccs_im = (float *)(ccs_base + (2 * y) * ccs_stride);
        ccs_re[0] = cmplxp[0][0]; // ReY_y,0
        ccs_im[0] = cmplxp[0][1]; // ImY_y,0
      }

      // y > rows / 2: the lower half-plane for X=0 is not written in CCS,
      // since it is fully reconstructed from the upper rows via conjugation.

      // 2. INNER COLUMNS (X = 1 ... half_cols - 1)
      // Linearly packed as Re/Im pairs into physical columns (2*x-1) and (2*x) of the current row y
      float * __restrict ccsp = (float *)(ccs_base + y * ccs_stride);
      for (int x = 1; x < half_cols; ++x) {
        ccsp[2 * x - 1] = cmplxp[x][0]; // Re
        ccsp[2 * x]     = cmplxp[x][1]; // Im
      }

      // 3. Nyquist column X = N/2 (if cols is even)
      // Packed vertically into last physical column (N-1) of the CCS matrix
      if (has_even_cols) {
        const int last_ccs_col = cols - 1;
        const int nq_x = cols / 2;
        if (y == 0) {
          float * __restrict ccs_re = (float *)(ccs_base + 0 * ccs_stride);
          ccs_re[last_ccs_col] = cmplxp[nq_x][0]; // ReY_0,N/2
        }
        else if (has_even_rows && y == rows / 2) {
          float * __restrict ccs_re = (float *)(ccs_base + (rows - 1) * ccs_stride);
          ccs_re[last_ccs_col] = cmplxp[nq_x][0]; // ReY_M/2,N/2
        }
        else if (y < half_rows) {
          float * __restrict ccs_re = (float *)(ccs_base + (2 * y - 1) * ccs_stride);
          float * __restrict ccs_im = (float *)(ccs_base + (2 * y) * ccs_stride);
          ccs_re[last_ccs_col] = cmplxp[nq_x][0]; // ReY_y,N/2
          ccs_im[last_ccs_col] = cmplxp[nq_x][1]; // ImY_y,N/2
        }
        // y > rows / 2: the lower half-plane for Nyquist is likewise shifted downwards.
      }
    }
  });

  return true;
}

/**
* @brief Performs element-wise multiplication of complex spectrum in OpenCV CCS format
*      by a general-purpose real amplitude filter by a
* @param[in] ccsInputSpectrum Input spectrum in OpenCV CCS format (size M x N, type CV_32FC1).
* @param[in] filter Real filter (size M x N, type CV_32FC1).
*      NOT in CSS format!!! Each pixel corresponds to a frequency.
*      DC component is at top left corner at index x = 0, y = 0.
* @param[out] ccsOutputSpectrum Output spectrum resulting from amplitude multiplication, OpenCV CCS format.
*/
bool fftMulSpectrumCCS(cv::InputArray _ccsInputSpectrum, const cv::Mat1f & filter,
    cv::OutputArray _ccsOutputSpectrum)
{
  if ( _ccsInputSpectrum.empty() || _ccsInputSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("Invalid argument: CV_32FC1 CCS spectrum is expected on input");
    return false;
  }

  if ( filter.size() != _ccsInputSpectrum.size() ) {
    CF_ERROR("Invalid argument: Filter size %dx%d not match to CCS spectrum size %dx%d",
        filter.cols, filter.rows, _ccsInputSpectrum.cols(), _ccsInputSpectrum.rows());
    return false;
  }

  if ( _ccsOutputSpectrum.fixedType() && _ccsOutputSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("Invalid argument: CCS output spectrum of CV_32FC1 type is expected for output");
    return false;
  }

  if( _ccsOutputSpectrum.fixedSize() && _ccsOutputSpectrum.size() != _ccsInputSpectrum.size() ) {
    CF_ERROR("Invalid argument: CCS output spectrum size %dx%d not match to input spectrum size %dx%d",
        _ccsOutputSpectrum.cols(), _ccsOutputSpectrum.rows(),
        _ccsInputSpectrum.cols(), _ccsInputSpectrum.rows());
    return false;
  }

  const cv::Size fftSize = _ccsInputSpectrum.size();
  const int rows = fftSize.height;
  const int cols = fftSize.width;
  const bool has_even_rows = (rows & 1) == 0;
  const bool has_even_cols = (cols & 1) == 0;

  const cv::Mat1f srcSpectrum = _ccsInputSpectrum.getMat();
  const uint8_t * src_base = srcSpectrum.ptr();
  const size_t src_stride = srcSpectrum.step;

  const uint8_t * filter_base = filter.ptr();
  const size_t filter_stride = filter.step;

  _ccsOutputSpectrum.create(fftSize, CV_32FC1);
  cv::Mat1f outSpectrum = _ccsOutputSpectrum.getMatRef();
  uint8_t * dst_base = outSpectrum.ptr();
  const size_t dst_stride = outSpectrum.step;

  parallel_for(0, rows, [=](const auto & range) {
    for( int y = rbegin(range); y < rend(range); ++y ) {
      const float * src_p = (const float *)(src_base + y * src_stride);
      const float * flt_p = (const float *)(filter_base + y * filter_stride);
      float * dst_p = (float *)(dst_base + y * dst_stride);

      // DC (X = 0) Packed vertically by frequency (Intel IPL)
      int true_freq_y = 0;
      if (y == 0) {
        true_freq_y = 0;
      }
      else if (has_even_rows && y == rows - 1) {
        true_freq_y = rows / 2;
      }
      else {
        true_freq_y = ((y & 1) == 0) ? (y / 2) : ((y + 1) / 2);
      }

      // Coefficient from the uncompressed filter for current frequency
      const float * flt_true_y = (const float *)(filter_base + true_freq_y * filter_stride);
      dst_p[0] = src_p[0] * flt_true_y[0];

      // INTERNAL COLUMNS (X = 1 ... half_cols - 1)
      // The CCS row strictly corresponds to frequency y; the filter is taken from the current flt_p[x]
      const int half_cols = (cols + 1) / 2;
      for (int x = 1; x < half_cols; ++x) {
        const float k_amplitude = flt_p[x];
        dst_p[2 * x - 1] = src_p[2 * x - 1] * k_amplitude; // Re
        dst_p[2 * x]  = src_p[2 * x] * k_amplitude;     // Im
      }

      // Nyquist column X = N/2 (if cols is even) packed vertically in the last physical CCS column (index = cols - 1)
      // The element last_ccs_col in row `y` is associated with the same vertical frequency `true_freq_y`
      // as the element in column 0 of the same row!
      if (has_even_cols) {
        const int last_ccs_col = cols - 1;
        const int x = cols / 2;
        dst_p[last_ccs_col] = src_p[last_ccs_col] * flt_true_y[x];
      }
    }
  });

  return true;
}

/**
 * Analytical computation of the 2D Complex CV_32FC2 spectrum V via 1D DFT of rows and columns.
 * Implements Virginie Moizan decomposition.
 * The src must be singke-channel real image
 */
bool fftComputeVSpectrumComplex(cv::InputArray _src, cv::OutputArray _complexSpectrum)
{
  if( _src.empty() || _src.channels() != 1 ) {
    CF_ERROR("Single-channel input image expected");
    return false;
  }

  const cv::Mat src = _src.getMat();
  const int rows = src.rows;
  const int cols = src.cols;

  // One-dimensional boundary differences
  cv::Mat drow, dcol;
  cv::subtract(src.row(0), src.row(rows - 1), drow, cv::noArray(), CV_32F);
  cv::subtract(src.col(0), src.col(cols - 1), dcol, cv::noArray(), CV_32F);

  // Two complex 1D DFT in CV_32FC2 of size 1 x N and 1 x M
  cv::Mat drow_complex, dcol_complex;
  cv::dft(drow, drow_complex, cv::DFT_COMPLEX_OUTPUT);
  cv::dft(dcol.t(), dcol_complex, cv::DFT_COMPLEX_OUTPUT);

  // Precompute trigonometric tables
  std::vector<float> cosx(cols), sinx(cols);
  std::vector<float> cosy(rows), siny(rows);
  for( int y = 0; y < rows; ++y ) {
    cosy[y] = std::cos(y * CV_2PI / rows);
    siny[y] = std::sin(y * CV_2PI / rows);
  }
  for( int x = 0; x < cols; ++x ) {
    cosx[x] = std::cos(x * CV_2PI / cols);
    sinx[x] = std::sin(x * CV_2PI / cols);
  }

  // Output complex 2D  CV_32FC2 matrix
  _complexSpectrum.create(rows, cols, CV_32FC2);
  cv::Mat2f V_COMPLEX = _complexSpectrum.getMatRef();
  uint8_t * const out_spec_base = V_COMPLEX.ptr();
  const size_t out_spec_stride = V_COMPLEX.step;

  const cv::Vec2f * drow_p = (const cv::Vec2f*) drow_complex.ptr();
  const cv::Vec2f * dcol_p = (const cv::Vec2f*) dcol_complex.ptr();

  const float * cosX = cosx.data();
  const float * cosY = cosy.data();
  const float * sinX = sinx.data();
  const float * sinY = siny.data();

  // Assembly of a complex 2D matrix
  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      cv::Vec2f * __restrict dstp = (cv::Vec2f*)(out_spec_base + y * out_spec_stride);

      // The contribution of columns dcol depends on the vertical frequency y
      const float re_c = dcol_p[y][0];
      const float im_c = dcol_p[y][1];

      // Vertical shift terms (1 - exp(-j * omega_y))
      const float sRe_Y = 1.0f - cosY[y];
      const float sIm_Y = sinY[y];

      for (int x = 0; x < cols; ++x) {
        // The contribution of rows drow depends on the horizontal frequency x
        const float re_r = drow_p[x][0];
        const float im_r = drow_p[x][1];

        // Horizontal shift terms (1 - exp(-j * omega_x))
        const float sRe_X = 1.0f - cosX[x];
        const float sIm_X = sinX[x];

        // Complex multiplication V = drow(x) * (1 - e^{-j wy}) + dcol(y) * (1 - e^{-j wx})
        // with sign inversion of the imaginary part (sines) to compensate for phase rotation.
        // The row difference contribution is modulated along the Y-axis
        // The column difference contribution is modulated along the X-axis
        const float rx_re = re_r * sRe_Y + im_r * sIm_Y;
        const float rx_im = -re_r * sIm_Y + im_r * sRe_Y;
        const float ry_re = re_c * sRe_X + im_c * sIm_X;
        const float ry_im = -re_c * sIm_X + im_c * sRe_X;

        // Final sum
        dstp[x][0] = rx_re + ry_re;
        dstp[x][1] = rx_im + ry_im;
      }
    }
  });

  // Reset singularity in DC component
  V_COMPLEX(0, 0) = cv::Vec2f(0.0f, 0.0f);

  return true;
}

// Analytical computation of the 2D CCS spectrum V via 1D DFT of rows and columns.
// Implements Virginie Moizan decomposition.
// Saves ~2.0 ms from ~15 ms on a 1024x1024 grayscale frame by eliminating the 2D DFT.
// The src must be single-channel real image
bool fftComputeVSpectrumCCS(cv::InputArray _src, cv::OutputArray ccsOutputVSpectrum)
{
  if( _src.empty() || _src.channels() != 1 ) {
    CF_ERROR("Single-channel input image expected");
    return false;
  }

  if ( ccsOutputVSpectrum.fixedType() && ccsOutputVSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("CV_32FC1 output V spectrum destination expected");
    return false;
  }

  if ( ccsOutputVSpectrum.fixedSize() && ccsOutputVSpectrum.size() != _src.size() ) {
    CF_ERROR("Output matrix size %dx%d not match to input image size %dx%d",
        ccsOutputVSpectrum.cols(), ccsOutputVSpectrum.rows(),
        _src.cols(), _src.rows() );
    return false;
  }

  const cv::Mat src = _src.getMat();
  const int rows = src.rows;
  const int cols = src.cols;
  const bool is_even_rows = (rows % 2 == 0);
  const bool is_even_x = (cols % 2 == 0);

  const int half_rows = (rows + 1) / 2;
  const int half_cols = (cols + 1) / 2;

  // One-dimensional boundary differences
  cv::Mat drow, dcol;
  cv::subtract(src.row(0), src.row(rows - 1), drow, cv::noArray(), CV_32F);
  cv::subtract(src.col(0), src.col(cols - 1), dcol, cv::noArray(), CV_32F);

  // Two complex 1D DFT in CV_32FC2 of size 1 x N and 1 x M
  cv::Mat drow_complex, dcol_complex;
  cv::dft(drow, drow_complex, cv::DFT_COMPLEX_OUTPUT);
  cv::dft(dcol.t(), dcol_complex, cv::DFT_COMPLEX_OUTPUT);

  // Precompute trigonometric tables
  std::vector<float> cosx(cols), sinx(cols);
  std::vector<float> cosy(rows), siny(rows);
  for (int y = 0; y < rows; ++y) {
    cosy[y] = std::cos(y * CV_2PI / rows);
    siny[y] = std::sin(y * CV_2PI / rows);
  }
  for (int x = 0; x < cols; ++x) {
    cosx[x] = std::cos(x * CV_2PI / cols);
    sinx[x] = std::sin(x * CV_2PI / cols);
  }

  // Output CCS packed CV_32FC1 spectrum
  ccsOutputVSpectrum.create(rows, cols, CV_32FC1);
  cv::Mat1f V_SPECTRUM = ccsOutputVSpectrum.getMatRef();
  uint8_t * const out_spec_base = V_SPECTRUM.ptr();
  const size_t out_spec_stride = V_SPECTRUM.step;

  const cv::Vec2f* drow_p = (const cv::Vec2f*)drow_complex.ptr();
  const cv::Vec2f* dcol_p = (const cv::Vec2f*)dcol_complex.ptr();
  const float * cosX = cosx.data();
  const float * cosY = cosy.data();
  const float * sinX = sinx.data();
  const float * sinY = siny.data();

  // Single-pass assembly of the analytical CCS matrix
  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict ccsp = (float * )(out_spec_base + y * out_spec_stride);

      // COLUMN X = 0  contribution strictly for the current physical row y
      int true_freq_y = 0;
      bool is_im_row = false;
      if (y == 0) {
        true_freq_y = 0;
      }
      else if (is_even_rows && y == rows - 1) {
        true_freq_y = rows / 2;
      }
      else {
        is_im_row = ((y & 1) == 0);
        true_freq_y = is_im_row ? (y / 2) : ((y + 1) / 2);
      }

      // The contribution of columns dcol_p at x=0 is 0.
      // All energy at x=0 comes from drow_p[0] (horizontal DC),
      // modulated by the vertical frequency true_freq_y.
      // Rotate the horizontal DC component of the rows around the vertical Y-axis
      const float re_r_0 = drow_p[0][0];
      const float im_r_0 = drow_p[0][1];
      const float rx_re_0 = re_r_0 * (1.0f - cosY[true_freq_y]) + im_r_0 * sinY[true_freq_y];
      const float rx_im_0 = -re_r_0 * sinY[true_freq_y] + im_r_0 * (1.0f - cosY[true_freq_y]);
      ccsp[0] = is_im_row ? rx_im_0 : rx_re_0;

      // INNER COLUMNS Linear pass along x from 1 to half_cols - 1
      const float re_c = dcol_p[y][0];
      const float im_c = dcol_p[y][1];
      const float cY = cosY[y];
      const float sY = sinY[y];

      for (int x = 1; x < half_cols; ++x) {
        // Row difference contribution modulated along Y
        // Column difference contribution modulated along X
        const float re_r = drow_p[x][0];
        const float im_r = drow_p[x][1];
        const float cX = cosX[x];
        const float sX = sinX[x];
        const float rx_re = re_r * (1.0f - cY) + im_r * sY;
        const float rx_im = -re_r * sY + im_r * (1.0f - cY);
        const float ry_re = re_c * (1.0f - cX) + im_c * sX;
        const float ry_im = -re_c * sX + im_c * (1.0f - cX);
        ccsp[2 * x - 1] = rx_re + ry_re;
        ccsp[2 * x]     = rx_im + ry_im;
      }

      // Nyquist column X = N/2 if cols is even, index is cols - 1
      if (is_even_x) {
        // For the horizontal Nyquist frequency, the contribution of dcol_p depends on true_freq_y
        // Extract Re/Im from dcol_p for true_freq_y and multiply by (1 - cos(pi)) = 2
        // The contribution of rows at the horizontal Nyquist frequency nq_x rotates along the Y-axis
        const int last_ccs_col = cols - 1;
        const int nq_x = cols / 2;
        const float re_c_nq = dcol_p[true_freq_y][0];
        const float im_c_nq = dcol_p[true_freq_y][1];
        const float ry_re_nq = re_c_nq * 2.0f;
        const float ry_im_nq = im_c_nq * 2.0f;
        const float re_r_nq = drow_p[nq_x][0];
        const float im_r_nq = drow_p[nq_x][1];
        const float rx_re_nq = re_r_nq * (1.0f - cosY[true_freq_y]) + im_r_nq * sinY[true_freq_y];
        const float rx_im_nq = -re_r_nq * sinY[true_freq_y] + im_r_nq * (1.0f - cosY[true_freq_y]);
        const float res_re = rx_re_nq + ry_re_nq;
        const float res_im = rx_im_nq + ry_im_nq;
        ccsp[last_ccs_col] = is_im_row ? res_im : res_re;
      }
    }
  });

  // Reset singularity in DC component
  V_SPECTRUM(0, 0) = 0.0f;
  return true;
}

/*
 * DFT with Periodic + Smooth Decomposition with CCS output.
 * Uses Virginie Moizan decomposition.
 * The Inverse Discrete Laplacian Filter VLAP must be prepared before this call with centerDC=false.
 *   const cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
 * The target fftSize (FFT padding) is defined by the VLAP.size()
 **/
bool fftPPSDecompositionCCS(cv::InputArray _src, const cv::Mat1f & VLAP,
    cv::OutputArray P_SPECTRUM, cv::OutputArray S_SPECTRUM,
    cv::OutputArray outputV_SPECTRUM)
{
  if( _src.empty() || _src.channels() != 1 ) {
    CF_ERROR("Single-channel input image expected");
    return false;
  }

  if ( _src.size() != VLAP.size() ) {
    CF_ERROR("Input image size %dx%d not match to VLAP filter size %dx%d",
        _src.cols(), _src.rows(), VLAP.cols, VLAP.rows);
    return false;
  }

  if ( P_SPECTRUM.fixedType() && P_SPECTRUM.type() != CV_32FC1 ) {
    CF_ERROR("CV_32FC1 output P spectrum destination expected");
    return false;
  }

  if ( P_SPECTRUM.fixedSize() && P_SPECTRUM.size() != _src.size() ) {
    CF_ERROR("Output P matrix size %dx%d not match to input image size %dx%d",
        P_SPECTRUM.cols(), P_SPECTRUM.rows(),
        _src.cols(), _src.rows() );
    return false;
  }

  if ( S_SPECTRUM.needed() ) {
    if ( S_SPECTRUM.fixedType() && S_SPECTRUM.type() != CV_32FC1 ) {
      CF_ERROR("CV_32FC1 output S spectrum destination expected");
      return false;
    }
    if ( S_SPECTRUM.fixedSize() && S_SPECTRUM.size() != _src.size() ) {
      CF_ERROR("Output S matrix size %dx%d not match to input image size %dx%d",
          S_SPECTRUM.cols(), S_SPECTRUM.rows(),
          _src.cols(), _src.rows() );
      return false;
    }
  }

  const cv::Size fftSize = VLAP.size();
  const cv::Mat SRC = _src.getMat();
  cv::Mat SRC_SPECTRUM, V_SPECTRUM;

  cv::dft(SRC, SRC_SPECTRUM, cv::DFT_REAL_OUTPUT);
  fftComputeVSpectrumCCS(SRC, V_SPECTRUM);

  if ( S_SPECTRUM.needed() ) {
    fftMulSpectrumCCS(V_SPECTRUM, VLAP, S_SPECTRUM);
    cv::subtract(SRC_SPECTRUM, S_SPECTRUM, P_SPECTRUM);
  }
  else {
    cv::Mat S_SPECTRUM_TMP;
    fftMulSpectrumCCS(V_SPECTRUM, VLAP, S_SPECTRUM_TMP);
    cv::subtract(SRC_SPECTRUM, S_SPECTRUM_TMP, P_SPECTRUM);
  }

  if ( outputV_SPECTRUM.needed() ) {
    outputV_SPECTRUM.move(V_SPECTRUM);
  }

  return true;
}

// DFT with Periodic + Smooth Decomposition with CCS output.
// Uses Virginie Moizan decomposition.
// The Inverse Discrete Laplacian Filter VLAP must be prepared before this call with centerDC=false.
// const cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
// The target fftSize (FFT padding) is defined by the VLAP.size()
bool fftPPSDecompositionCCS(cv::InputArray _src, const cv::Mat1f & VLAP,
    std::vector<cv::Mat1f> * P_SPECTRUMS, std::vector<cv::Mat1f> * S_SPECTRUMS)
{
  if( _src.empty() ) {
    CF_ERROR("Single-channel input image expected");
    return false;
  }

  if ( _src.size() != VLAP.size() ) {
    CF_ERROR("Input image size %dx%d not match to VLAP filter size %dx%d",
        _src.cols(), _src.rows(), VLAP.cols, VLAP.rows);
    return false;
  }

  const cv::Size fftSize = VLAP.size();
  const cv::Mat SRC = _src.getMat();
  const int cn = _src.channels();

  std::vector<cv::Mat1f> src_channels(cn);
  cv::Mat SRC_SPECTRUM, V_SPECTRUM, S_TMP;

  if ( cn == 1 ) {
    src_channels[0] = SRC;
  }
  else {
    cv::split(SRC, src_channels);
  }

  P_SPECTRUMS->resize(cn);
  if ( S_SPECTRUMS != nullptr ) {
    S_SPECTRUMS->resize(cn);
  }

  for ( int i = 0; i < cn; ++i ) {
    cv::dft(src_channels[i], SRC_SPECTRUM, cv::DFT_REAL_OUTPUT);
    fftComputeVSpectrumCCS(src_channels[i], V_SPECTRUM);

    if ( S_SPECTRUMS != nullptr ) {
      fftMulSpectrumCCS(V_SPECTRUM, VLAP, S_SPECTRUMS->at(i));
      cv::subtract(SRC_SPECTRUM, S_SPECTRUMS->at(i), P_SPECTRUMS->at(i));
    }
    else {
      fftMulSpectrumCCS(V_SPECTRUM, VLAP, S_TMP);
      cv::subtract(SRC_SPECTRUM, S_TMP, P_SPECTRUMS->at(i));
    }
  }

  return true;
}

// DFT with Periodic + Smooth Decomposition with CCS output.
// Uses Virginie Moizan decomposition.
// The Inverse Discrete Laplacian Filter VLAP must be prepared before this call with centerDC=false.
// const cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
// The target fftSize (FFT padding) is defined by the VLAP.size()
bool fftPPSDecompositionCCSPlanes(const std::vector<cv::Mat> & planes, const cv::Mat1f & VLAP,
    std::vector<cv::Mat1f> * P_SPECTRUMS, std::vector<cv::Mat1f> * S_SPECTRUMS)
{
  INSTRUMENT_REGION("");

  if( planes.empty() ) {
    CF_ERROR("Input argument is empty");
    return false;
  }

  const cv::Size fftSize = VLAP.size();
  if ( fftSize.empty() ) {
    CF_ERROR("Bad VLAP: empty");
    return false;
  }

  const int cn = (int)planes.size();
  for ( int c = 0; c < cn; ++c ) {
    if ( planes[c].size() != fftSize ) {
      CF_ERROR("Bad image size on plane %d: %dx%d not match to VLAP filter size %dx%d",
          c, planes[c].cols, planes[c].rows, VLAP.cols, VLAP.rows);
      return false;
    }
  }

  P_SPECTRUMS->resize(cn);
  if ( S_SPECTRUMS != nullptr ) {
    S_SPECTRUMS->resize(cn);
  }

  cv::Mat SRC_SPECTRUM, V_SPECTRUM, S_TMP;

  for ( int c = 0; c < cn; ++c ) {
    cv::dft(planes[c], SRC_SPECTRUM, cv::DFT_REAL_OUTPUT);
    fftComputeVSpectrumCCS(planes[c], V_SPECTRUM);

    if ( S_SPECTRUMS != nullptr ) {
      fftMulSpectrumCCS(V_SPECTRUM, VLAP, S_SPECTRUMS->at(c));
      cv::subtract(SRC_SPECTRUM, S_SPECTRUMS->at(c), P_SPECTRUMS->at(c));
    }
    else {
      fftMulSpectrumCCS(V_SPECTRUM, VLAP, S_TMP);
      cv::subtract(SRC_SPECTRUM, S_TMP, P_SPECTRUMS->at(c));
    }
  }

  return true;
}

/**
 * @brief Computes the weighted phase correlation cross of two spectra packed in OpenCV CCS format.
 *
 * This function performs element-wise cross-multiplication of two spectra with conjugation of the
 * second spectrum, followed by phase whitening (amplitude normalization) and application of a real bandpass filter.
 * Mathematically, for each frequency it computes: \f$ DST = filter \cdot \frac{S_1 \cdot S_2^*}{|S_1 \cdot S_2^*|} \f$
 *
 * @note The algorithm is optimized for multi-threaded execution (via parallel_for) and operates directly
 * on the packed OpenCV CCS (Complex Conjugate Symmetrical) format. This eliminates redundant memory
 * allocations for full complex matrices.
 *
 * @note **Normalization & Correlation Peak Mechanics:**
 * If the input `filter` is pre-normalized using the L1-norm to unity (i.e., sum(gw) = 1), then the subsequent
 * call to inverse Fourier transform `cv::idft(..., cv::DFT_REAL_OUTPUT)` WITHOUT the `cv::DFT_SCALE` flag
 * will yield a peak value of strictly **1.0** on the autocorrelation map (given a perfect match). This occurs
 * because the \f$1/N\f$ scale introduced by the filter's L1-normalization perfectly cancels out the internal \f$N\f$
 * scaling factor inherent to OpenCV's unscaled IDFT.
 *
 * @param[in] ccsSpectrum1 First input image spectrum in OpenCV CCS format (CV_32FC1, real matrix).
 * @param[in] ccsSpectrum2 Second input image spectrum in OpenCV CCS format (CV_32FC1, real matrix).
 * @param[in] filter Real bandpass filter matrix (frequency weights) matching the size of the input spectra.
 * @param[out] _crossSpectrum Output filtered cross-spectrum in CCS format (CV_32FC1).
 *
 * @return Returns false in case of a size mismatch error.
 */
bool fftCrossSpectrumPhaseCorrelateWeightedCCS(cv::InputArray _ccsSpectrum1, cv::InputArray _ccsSpectrum2,
    const cv::Mat1f & filter, cv::OutputArray _crossSpectrum)
{
  if( _ccsSpectrum1.empty() || _ccsSpectrum1.type() != CV_32FC1 ) {
    CF_ERROR("Non-empty CCS packed spectrum 1 of type CV_32FC1 is expected on input");
    return false;
  }
  if( _ccsSpectrum2.empty() || _ccsSpectrum2.type() != CV_32FC1 ) {
    CF_ERROR("Non-empty CCS packed spectrum 2 of type CV_32FC1 is expected on input");
    return false;
  }

  if ( _ccsSpectrum1.size() != _ccsSpectrum2.size() ) {
    CF_ERROR("Sizes of input spectrums %dx%d and %dx%d not match",
        _ccsSpectrum1.cols(), _ccsSpectrum1.rows(),
        _ccsSpectrum2.cols(), _ccsSpectrum2.rows() );
    return false;
  }

  if ( filter.size() != _ccsSpectrum1.size() ) {
    CF_ERROR("Filterr size %dx%d not match to spectrum size %dx%d",
        filter.cols, filter.rows, _ccsSpectrum1.cols(), _ccsSpectrum1.rows() );
    return false;
  }

  if ( _crossSpectrum.fixedType() && _crossSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("Invalid argument: CCS output spectrum of CV_32FC1 type is expected for output");
    return false;
  }

  if( _crossSpectrum.fixedSize() && _crossSpectrum.size() != filter.size() ) {
    CF_ERROR("Invalid argument: CCS output spectrum size %dx%d not match to filyer size %dx%d",
        _crossSpectrum.cols(), _crossSpectrum.rows(),
        filter.cols, filter.rows);
    return false;
  }

  const cv::Size fftSize = filter.size();
  const int rows = fftSize.height;
  const int cols = fftSize.width;
  const bool has_even_rows = (rows & 1) == 0;
  const bool has_even_cols = (cols & 1) == 0;

  const cv::Mat1f ccs1 = _ccsSpectrum1.getMat();
  const uint8_t * ccs1_base = ccs1.ptr();
  const size_t ccs1_stride = ccs1.step;

  const cv::Mat1f ccs2 = _ccsSpectrum2.getMat();
  const uint8_t * ccs2_base = ccs2.ptr();
  const size_t ccs2_stride = ccs2.step;

  const uint8_t * flt_base  = filter.ptr();
  const size_t flt_stride  = filter.step;

  _crossSpectrum.create(rows, cols, CV_32FC1);
  cv::Mat1f crossSpectrum = _crossSpectrum.getMatRef();
  uint8_t * cross_base = crossSpectrum.ptr();
  const size_t cross_stride = crossSpectrum.step;

  static constexpr float safe_min =
      std::numeric_limits<float>::min();

  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      const float * srcp1 = (const float *)(ccs1_base + y * ccs1_stride);
      const float * srcp2 = (const float *)(ccs2_base + y * ccs2_stride);
      const float * fltp  = (const float *)(flt_base  + y * flt_stride);
      float * __restrict cross = (float *)(cross_base + y * cross_stride);

      // DC(X = 0) Vertical 1D CCS according to OpenCV code
      if ( true ) {
        float cross_value = 0.0f;

        if (y == 0) {
          // Pure real DC component (0,0). Single-component whitening.
          const float a1 = srcp1[0];
          const float a2 = srcp2[0];
          const float re = a1 * a2;
          const float mag = std::abs(re);
          if (mag > safe_min) {
            const float gw = ((const float *)(flt_base + 0 * flt_stride))[0];
            cross_value = re * gw / mag;
          }
        }
        else if (has_even_rows && y == rows - 1) {
          // Pure real vertical Nyquist (rows/2, 0) in the last row of the CCS.
          const float a1 = srcp1[0];
          const float a2 = srcp2[0];
          const float re = a1 * a2;
          const float mag = std::abs(re);
          if (mag > safe_min) {
            const float gw = ((const float *)(flt_base + (rows / 2) * flt_stride))[0];
            cross_value = re * gw / mag;
          }
        }
        else {
          // Intermediate complex frequencies on the X=0 axis.
          // Cross-multiplication with conjugation (like in mulSpectrums)
          const bool is_im_row = ((y & 1) == 0); // Even rows (2,4,6...) store Im
          const int r_idx = is_im_row ? (y - 1) : y;
          const int i_idx = is_im_row ? y : (y + 1);
          const int true_freq_y = (r_idx + 1) / 2;
          const float a1 = ((const float *)(ccs1_base + r_idx * ccs1_stride))[0];
          const float b1 = ((const float *)(ccs1_base + i_idx * ccs1_stride))[0];
          const float a2 = ((const float *)(ccs2_base + r_idx * ccs2_stride))[0];
          const float b2 = ((const float *)(ccs2_base + i_idx * ccs2_stride))[0];
          const float re = (a1 * a2 + b1 * b2);
          const float im = (b1 * a2 - a1 * b2);
          const float mag = std::sqrt(re * re + im * im);
          if (mag > safe_min) {
            const float gw = ((const float *)(flt_base + true_freq_y * flt_stride))[0];
            cross_value = (is_im_row ? im : re) * gw / mag;
          }
        }

        cross[0] = cross_value;
      }

      // Internal complex columns (X = 1 ... half_cols - 1)
      const int half_cols = (cols + 1) / 2;
      for (int x = 1; x < half_cols; ++x) {
        const float gw = fltp[x];
        const float a1 = srcp1[2 * x - 1];
        const float b1 = srcp1[2 * x];
        const float a2 = srcp2[2 * x - 1];
        const float b2 = srcp2[2 * x];
        const float re = (a1 * a2 + b1 * b2);
        const float im = (b1 * a2 - a1 * b2);
        const float mag2 = (re * re + im * im);
        if (mag2 > safe_min) {
          const float scale = gw / std::sqrt(mag2);
          cross[2 * x - 1] = re * scale;
          cross[2 * x]     = im * scale;
        }
        else {
          cross[2 * x - 1] = 0.0f;
          cross[2 * x]     = 0.0f;
        }
      }

      // Nyquist column X = N/2 If cols is even, the physical index is cols - 1.
      if (has_even_cols) {
        const int last_ccs_col = cols - 1;
        const int x = cols / 2;
        float cross_value = 0.0f;

        if (y == 0) {
          const float a1 = srcp1[last_ccs_col];
          const float a2 = srcp2[last_ccs_col];
          const float re = a1 * a2;
          const float mag = std::abs(re);
          if (mag > safe_min) {
            const float gw = ((const float *)(flt_base + 0 * flt_stride))[x];
            cross_value = re * gw / mag;
          }
        }
        else if (has_even_rows && y == rows - 1) {
          const float a1 = srcp1[last_ccs_col];
          const float a2 = srcp2[last_ccs_col];
          const float re = a1 * a2;
          const float mag = std::abs(re);
          if (mag > safe_min) {
            const float gw = ((const float *)(flt_base + (rows / 2) * flt_stride))[x];
            cross_value = re * gw / mag;
          }
        }
        else {
          const bool is_im_row = ((y & 1) == 0);
          const int r_idx = is_im_row ? (y - 1) : y;
          const int i_idx = is_im_row ? y : (y + 1);
          const int true_freq_y = (r_idx + 1) / 2;
          const float a1 = ((const float *)(ccs1_base + r_idx * ccs1_stride))[last_ccs_col];
          const float b1 = ((const float *)(ccs1_base + i_idx * ccs1_stride))[last_ccs_col];
          const float a2 = ((const float *)(ccs2_base + r_idx * ccs2_stride))[last_ccs_col];
          const float b2 = ((const float *)(ccs2_base + i_idx * ccs2_stride))[last_ccs_col];
          const float re = (a1 * a2 + b1 * b2);
          const float im = (b1 * a2 - a1 * b2);
          const float mag = std::sqrt(re * re + im * im);
          if (mag > safe_min) {
            const float gw = ((const float *)(flt_base + true_freq_y * flt_stride))[x];
            cross_value = (is_im_row ? im : re) * gw / mag;
          }
        }
        cross[last_ccs_col] = cross_value;
      }
    }
  });

  return true;
}


/**
 * @brief Computes the bandpass-filtered autocorrelation spectrum (energy map) in CCS format.
 *
 * This function performs an in-place-like multiplication of a complex CCS spectrum by a real-valued
 * amplitude filter. It automatically handles the internal layout symmetry of the OpenCV CCS format
 * and eliminates the imaginary components, as the autocorrelation of a real signal is strictly real.
 *
 * @param[in] ccsSpectrum Input spectrum matrix of type CV_32FC1, packed in OpenCV's CCS
 *                        (Complex Conjugate Symmetric) format (e.g., generated by cv::dft with DFT_REAL_OUTPUT).
 *
 * @param[in] filter Precomputed amplitude filter matrix of type CV_32FC1 and of the EXACT SAME SIZE
 *                   as ccsSpectrum.
 *                   CRITICAL LAYOUT REQUIREMENTS:
 *                   - Must be in NORMAL UNPACKED 2D FFT layout (NOT a CCS matrix).
 *                   - Contains only the REAL PLANE (magnitude/amplitude weights).
 *                   - DC component (zero frequency) must be located strictly at the top-left pixel (0,0).
 *                   - High frequencies (Nyquist) must converge toward the center of the matrix (rows/2, cols/2).
 *                   - All 4 quadrants must be explicitly present and symmetric relative to the Nyquist axes.
 *                   - To bake in an embedded fftShift, multiply the filter values by the alternating
 *                     sign mask beforehand during its generation.
 *
 * @param[out] _autoCrossSpectrum Output filtered autocorrelation spectrum matrix of type CV_32FC1
 *                                in CCS format.
 *
 * @return double The total integrated bandpass energy of the filtered spectrum,
 */
double fftAutoCrossSpectrumWeightedCCS(cv::InputArray _ccsSpectrum, const cv::Mat1f & filter,
    cv::OutputArray _autoCrossSpectrum)
{
  if( _ccsSpectrum.empty() || _ccsSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("Non-empty CCS packed input spectrum of type CV_32FC1 is expected");
    return 0;
  }

  if( filter.size() != _ccsSpectrum.size() ) {
    CF_ERROR("Filter size %dx%d does not match spectrum size %dx%d",
        filter.cols, filter.rows, _ccsSpectrum.cols(), _ccsSpectrum.rows());
    return 0;
  }

  if( _autoCrossSpectrum.fixedType() && _autoCrossSpectrum.type() != CV_32FC1 ) {
    CF_ERROR("Invalid argument: CCS output spectrum of CV_32FC1 type is expected for output");
    return 0;
  }

  if( _autoCrossSpectrum.fixedSize() && _autoCrossSpectrum.size() != filter.size() ) {
    CF_ERROR("Invalid argument: CCS output spectrum size %dx%d does not match filter size %dx%d",
        _autoCrossSpectrum.cols(), _autoCrossSpectrum.rows(), filter.cols, filter.rows);
    return 0;
  }

  const cv::Size fftSize = filter.size();
  const int rows = fftSize.height;
  const int cols = fftSize.width;
  const bool has_even_rows = (rows & 1) == 0;
  const bool has_even_cols = (cols & 1) == 0;

  const cv::Mat1f ccs = _ccsSpectrum.getMat();
  const uint8_t * ccs_base = ccs.ptr();
  const size_t ccs_stride = ccs.step;

  const uint8_t * flt_base = filter.ptr();
  const size_t flt_stride = filter.step;

  _autoCrossSpectrum.create(rows, cols, CV_32FC1);
  cv::Mat1f autoCrossSpectrum = _autoCrossSpectrum.getMatRef();
  uint8_t * cross_base = autoCrossSpectrum.ptr();
  const size_t cross_stride = autoCrossSpectrum.step;

  alignas(std::hardware_destructive_interference_size)
    std::atomic<float> total_energy(0.0f);

  parallel_for(0, rows, [=, &total_energy](const auto & range) {
    float local_energy = 0.0f;

    for (int y = rbegin(range); y < rend(range); ++y) {
      const float * srcp = (const float *)(ccs_base + y * ccs_stride);
      const float * fltp = (const float *)(flt_base + y * flt_stride);
      float * __restrict cross = (float *)(cross_base + y * cross_stride);

      // X = 0 Pairwise Re/Im parsing by row (OpenCV CCS format)
      if ( true ) {
        float mag2 = 0.0f;

        if (y == 0) {
          // DC component (0,0) — purely real
          const float re = srcp[0];
          const float gw = ((const float *)(flt_base + 0 * flt_stride))[0];
          mag2 = gw * re * re;
          local_energy += mag2 * mag2;
        }
        else if (has_even_rows && y == rows - 1) {
          // Vertical Nyquist (rows/2, 0) — purely real
          const float re = srcp[0];
          const float gw = ((const float *)(flt_base + (rows / 2) * flt_stride))[0];
          mag2 = gw * re * re;
          local_energy += 2 * mag2 * mag2;
        }
        else if ( y & 1 ) {
          // Intermediate frequencies for X-axis = 0. Link the Re row and the Im row.
          // Uneven rows store Re
          const int r_idx = (y + 0);
          const int i_idx = (y + 1);
          const int true_freq_y = (r_idx + 1) / 2;
          const float re = ((const float *)(ccs_base + r_idx * ccs_stride))[0];// Re
          const float im = ((const float *)(ccs_base + i_idx * ccs_stride))[0];// Im
          const float gw = ((const float *)(flt_base + true_freq_y * flt_stride))[0];
          mag2 = gw * (re * re + im * im);
          local_energy += 2 * mag2 * mag2;
        }
        cross[0] = mag2;
      }

      // internal complex columns (X = 1 ... half_cols - 1)
      const int half_cols = (cols + 1) / 2;
      for (int x = 1; x < half_cols; ++x) {
        const float gw = fltp[x];
        const float re = srcp[2 * x - 1];
        const float im = srcp[2 * x];
        const float mag2 = gw * (re * re + im * im);
        local_energy += 2 * mag2 * mag2;
        cross[2 * x - 1] = mag2;
        cross[2 * x] = 0.0f;
      }

      // Nyquist column X = N/2 If cols is even, the physical index is cols - 1.
      if (has_even_cols) {
        const int last_ccs_col = cols - 1;
        const int x = cols / 2;
        float mag2 = 0.0f;

        if (y == 0) {
          const float re = srcp[last_ccs_col];
          const float gw = ((const float *)(flt_base + 0 * flt_stride))[x];
          mag2 = gw * re * re;
        }
        else if (has_even_rows && y == rows - 1) {
          const float re = srcp[last_ccs_col];
          const float gw = ((const float *)(flt_base + (rows / 2) * flt_stride))[x];
          mag2 = re * re * gw;
        }
        else if ( (y & 1)) {
          // Uneven rows store Re
          const int r_idx = (y + 0);
          const int i_idx = (y + 1);
          const int true_freq_y = (r_idx + 1) / 2;
          const float re = ((const float *)(ccs_base + r_idx * ccs_stride))[last_ccs_col];
          const float im = ((const float *)(ccs_base + i_idx * ccs_stride))[last_ccs_col];
          const float gw = ((const float *)(flt_base + true_freq_y * flt_stride))[x];
          mag2 = gw * (re * re + im * im);
        }
        local_energy += mag2 * mag2;
        cross[last_ccs_col] = mag2;
      }
    }

    float current = total_energy.load(std::memory_order_relaxed);
    while (!total_energy.compare_exchange_weak(current, current + local_energy,
            std::memory_order_relaxed));
  });

  return total_energy.load();
}

