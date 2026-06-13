/*
 * fft.cc
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#include "fft.h"
#include <tbb/tbb.h>
#include <core/debug.h>

static inline double hyp(double x, double y)
{
  return sqrt(x * x + y * y);
}

cv::Size fftGetOptimalSize(const cv::Size & srcSize, cv::Size psfSize, cv::Rect * roirc, bool forceEvenSize)
{
  const cv::Size imageSize = psfSize.empty() ? srcSize : srcSize + psfSize * 2;
  int w, h;

  if ( !forceEvenSize ) {
    w = cv::getOptimalDFTSize(imageSize.width);
  }
  else {
    for ( int i = 0;; ++i ) {
      if ( !((w = cv::getOptimalDFTSize(imageSize.width + i)) & 0x7) ) {
        break;
      }
    }
  }

  if ( !forceEvenSize ) {
    h = cv::getOptimalDFTSize(imageSize.height);
  }
  else {
    for ( int i = 0;; ++i ) {
      if ( !((h = cv::getOptimalDFTSize(imageSize.height + i)) & 0x7) ) {
        break;
      }
    }
  }

  const cv::Size fftSize(w, h);
  if( roirc ) {
    const int border_top = (fftSize.height - srcSize.height) / 2;
    const int border_bottom = (fftSize.height - srcSize.height - border_top);
    const int border_left = (fftSize.width - srcSize.width) / 2;
    const int border_right = (fftSize.width - srcSize.width - border_left);
    *roirc = cv::Rect(border_left, border_top, srcSize.width, srcSize.height);
  }

  return fftSize;
}


bool fftCopyMakeBorder(cv::InputArray src, cv::OutputArray dst, const cv::Size & fftSize,
    cv::Rect * outrc)
{
  const cv::Size src_size = src.size();

  if ( fftSize.width < src_size.width || fftSize.height < src_size.height ) {
    CF_ERROR("Invalid argument: fftSize (%dx%d) must be >= src.size() (%dx%d)",
        fftSize.width, fftSize.height, src_size.width, src_size.height );
    return false;
  }

  if ( src_size.width == fftSize.width && src_size.height == fftSize.height ) {
    if ( src.getMat().data != dst.getMatRef().data ) {
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
    cv::copyMakeBorder(src, dst, border_top, border_bottom, border_left, border_right, cv::BORDER_REFLECT);
    if ( outrc ) {
      * outrc = cv::Rect(border_left, border_top, src_size.width, src_size.height);
    }
  }

  return true;
}

void fftImageToSpectrum(cv::InputArray src, std::vector<cv::Mat2f> & complex_channels)
{
  const int cn = src.channels();
  complex_channels.resize(cn);

  if ( cn == 1 ) {
    cv::dft(src, complex_channels[0], cv::DFT_COMPLEX_OUTPUT);
  }
  else {
    cv::Mat channels[cn];
    cv::split(src.getMat(), channels);

    for ( int i = 0; i < cn; ++i ) {
      cv::dft(channels[i], complex_channels[i], cv::DFT_COMPLEX_OUTPUT);
    }
  }
}


bool fftImageToSpectrum(cv::InputArray _src, cv::OutputArray _dst, const cv::Size & fftSize,
    bool swapQuadrants)
{
  // Single-channel CV_32F image is expected on input
  if ( _src.type() != CV_32FC1 ) {
    CF_ERROR("Invalid argument: Single-channel CV_32F image is expected on input");
    return false;
  }

  cv::Mat src_padded;
  if ( _src.size() == fftSize ) {
    src_padded = _src.getMat();
  }
  else {
    fftCopyMakeBorder(_src, src_padded, fftSize);
  }

  const cv::Mat src_planes[] = { src_padded,
      cv::Mat::zeros(_src.size(), _src.depth())
  };

  // complex_I at the output will contain a spectrum in CV_32FC2 format
  cv::Mat complex_image;
  cv::merge(src_planes, 2, complex_image);
  cv::dft(complex_image, _dst);
  if ( swapQuadrants ) {
    fftSwapQuadrants(_dst.getMatRef());
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


void fftImageToSpectrum(cv::InputArray image,
    std::vector<cv::Mat2f> & complex_channels,
    const cv::Size & psfSize,
    cv::Rect * outrc)
{
  if ( psfSize.empty() ) {

    fftImageToSpectrum(image,
        complex_channels);

    if ( outrc ) {
      *outrc = cv::Rect(0, 0,
          image.cols(),
          image.rows());
    }

  }
  else {

    cv::Mat tmp;
    cv::Rect rc;

    fftCopyMakeBorder(image, tmp,
        fftGetOptimalSize(image.size(),
            psfSize),
            &rc);

    fftImageToSpectrum(tmp,
        complex_channels);

    if ( outrc ) {
      *outrc = rc;
    }
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
  cv::Mat1f dst(size);

  cv::parallel_for_(cv::Range(0, size.height),
      [&, size](const auto & range) {
        for ( int y = range.start; y < range.end; ++y ) {
          const float * srcp = (const float * )src[y];
          float * __restrict dstp = dst[y];
          for ( int x = 0; x < size.width; ++x, srcp += 2 ) {
            * dstp ++ = srcp[0] * srcp[0] + srcp[1] * srcp[1];
          }
        }
      });

  _dst.move(dst);

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

  tbb::parallel_for(0, src.rows,
      [&src, &dst](int y) {
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

  tbb::parallel_for(0, src.rows,
      [&]( int y) {
        for ( int x = 0; x < csrc.cols; ++x ) {
          cmag[y][x] = sqrt(csrc[y][x][0]*csrc[y][x][0] + csrc[y][x][1]*csrc[y][x][1] );
          cphase[y][x] = atan2(csrc[y][x][1], csrc[y][x][0]);
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

  tbb::parallel_for(0, cdst.rows,
      [&]( int y) {
        for ( int x = 0; x < cdst.cols; ++x ) {
          double sa, ca;
          sincos(cphase[y][x], &sa, &ca);
          cdst[y][x][0] = cmag[y][x] * ca;
          cdst[y][x][1] = cmag[y][x] * sa;
        }
      });

  return true;
}

bool fftRadialProfile(const cv::Mat1f & spectrum, std::vector<float> & output_profile, bool includeCorners)
{
  const int cx = spectrum.cols / 2;
  const int cy = spectrum.rows / 2;

  // Deformation (stretchimg) coefficients: convert spectral pixels into dimensionless radial units
  const double R = std::min(cx, cy);
  const double scaleX = spectrum.cols / (2.0 * R);
  const double scaleY = spectrum.rows / (2.0 * R);

  // Maximum radius in physical pixels of the matrix
  const double maxRadiusPx = includeCorners ? std::sqrt(cx * cx + cy * cy) : std::min(cx, cy);
  const int numBins = std::max(1, static_cast<int>(std::ceil(maxRadiusPx)));
  const double invMaxRadiusPx = (numBins - 1) / maxRadiusPx;


  // Sampling step: how many bins are there per 1 pixel of the spectrum radius

  std::vector<double> radialSum(numBins, 0.0);
  std::vector<int> radialCount(numBins, 0);

  // Scan only the upper half of the matrix up to the central row (y = cy) inclusive
  for( int y = 0; y <= cy; ++y ) {
    const float * srcp = spectrum[y];

    // normalize the vertical step
    const double dy = (y - cy) * scaleY;
    const double dy2 = dy * dy;

    const int xmax = (y == cy) ? cx : spectrum.cols;
    for( int x = 0; x < xmax; ++x ) {
      // normalize the horizontal step
      const double dx = (x - cx) * scaleX;
      const double dx2 = dx * dx;

      // True isotropic frequency radius consistent with space
      const double radiusPx = std::sqrt(dx2 + dy2);
      const int bin = static_cast<int>(radiusPx * invMaxRadiusPx);

      if( bin >= 0 && bin < numBins ) {
        // Symmetry:
        // For regular rows (y < cy) — always 2 (top mirrors bottom).
        // For the center row (y == cy) — 2 for all pixels except the very center (left mirrors right).
        // For the DC component itself (y == cy, x == cx) — 1, since it is unique.
        const int weight = (y < cy || x != cx) ? 2 : 1;
        radialSum[bin] += (srcp[x] * weight);
        radialCount[bin] += weight;
      }
    }
  }

  output_profile.resize(numBins);
  for( int i = 0; i < numBins; ++i ) {
    output_profile[i] = (float)(radialCount[i] > 0 ? radialSum[i] / radialCount[i] : 0);
  }

  return true;
}

bool fftRadialProfile(const cv::Mat1f & spectrum, cv::Mat1f & output_profile, bool includeCorners)
{
  // Maximum radius in physical pixels of the matrix
  const int cx = spectrum.cols / 2;
  const int cy = spectrum.rows / 2;
  const double R = includeCorners ? std::sqrt(cx * cx + cy * cy) : std::min(cx, cy);
  const int numBins = std::max(1, (int)(R));

  // Deformation (stretching) coefficients: convert spectral pixels into dimensionless radial units
  const double scaleX = R / cx;
  const double scaleY = R / cy;

  // Histogram accumulators
  std::vector<double> radialSum(numBins, 0.0);
  std::vector<double> radialCount(numBins, 0.0);

  // Scan only the upper half of the matrix up to the central row (y = cy) inclusive
  for( int y = 0; y <= cy; ++y ) {
    const float * srcp = spectrum[y];

    // normalize the vertical step
    const double dy = (y - cy) * scaleY;
    const double dy2 = dy * dy;

    const int xmax = (y == cy) ? (cx + 1) : spectrum.cols;
    for( int x = 0; x < xmax; ++x ) {
      // normalize the horizontal step
      const double dx = (x - cx) * scaleX;
      const double dx2 = dx * dx;

      // True isotropic frequency radius consistent with space
      const double r = std::sqrt(dx2 + dy2);
      const int bin = (int)(r);
      if( bin < numBins ) {
        // Symmetry:
        // For regular rows (y < cy) — always 2 (top mirrors bottom).
        // For the center row (y == cy) — 2 for all pixels except the very center (left mirrors right).
        // For the DC component itself (y == cy, x == cx) — 1, since it is unique.
        const int weight = (y < cy || x != cx) ? 2 : 1;
        radialSum[bin] += srcp[x] * weight;
        radialCount[bin] += weight;
      }
    }
  }

  output_profile.create(1, numBins);
  float * __restrict dstp = output_profile[0];
  for( int i = 0; i < numBins; ++i ) {
    dstp[i] = (float)(radialCount[i] > 0 ? radialSum[i] / radialCount[i] : 0.0);
  }

  return true;
}

void fftRadialProfileToImage(const cv::Mat1f & radialProfile,
    const cv::Size & outputImageSize,
    bool cornersIncluded,
    cv::Mat1f & outputImage)
{
  const cv::Size & size = outputImageSize;
  outputImage.create(size);

  const double cx = size.width / 2;
  const double cy = size.height / 2;
  const double R = cornersIncluded ? std::sqrt(cx * cx + cy * cy) : std::min(cx, cy);
  const int numBins = radialProfile.cols;

  // Deformation (stretching) coefficients: convert spectral pixels into dimensionless radial units
  const double scaleX = R / cx;
  const double scaleY = R / cy;

  cv::parallel_for_(cv::Range(0, size.height),
      [=, &radialProfile, &outputImage](const cv::Range & range) {

        const float * bins = radialProfile[0];

        for (int y = range.start; y < range.end; ++y) {
          float * __restrict dstp = outputImage[y];

          // normalize the vertical step
          const double dy = (y - cy) * scaleY;
          const double dy2 = dy * dy;

          for (int x = 0; x < size.width; ++x) {

            // normalize the horizontal step
            const double dx = (x - cx) * scaleX;
            const double dx2 = dx * dx;

            // Radial radius in pixels (metrically perfect circle)
            const double r = std::sqrt(dx2 + dy2);
            const int bin = static_cast<int>(r); //  / R
            if (bin >= 0 && bin < numBins) {
              dstp[x] = bins[bin];
            }
            else {
              // If cornersIncluded = false, all pixels from the corner zones of the matrix will fall outside numBins.
              dstp[x] = 0; /*  can be also radialProfile.back(); */
            }
          }
        }
    });
}

void fftRadialPolySharp(cv::InputArray src, cv::OutputArray dst,
    const std::vector<double> & coeffs,
    std::vector<double> & profile_before,
    std::vector<double> & profile_after,
    std::vector<double> & profile_poly)
{
  cv::Mat image;
  std::vector<cv::Mat> channels;
  std::vector<int> profile_counter;
  cv::Rect rc;

  constexpr bool preserve_l2_norm = false;
  double saved_image_norm = 1;

  if ( preserve_l2_norm ) {
    saved_image_norm = cv::norm(src, cv::NORM_L2);
  }


  const cv::Size fftSize =
      fftGetOptimalSize(src.size(),
          cv::Size(64, 64),
          nullptr,
          true);

  if ( src.size() == fftSize ) {
    src.copyTo(image);
  }
  else {
    fftCopyMakeBorder(src, image, fftSize, &rc);
  }


  if ( image.channels() == 1 ) {
    channels.emplace_back(image);
  }
  else {
    cv::split(image, channels);
  }

  const cv::Point C(fftSize.width / 2, fftSize.height / 2);
  const int Rmax = (int) ceil(hyp(C.x, C.y));

  profile_before.clear();
  profile_before.resize(Rmax, 0);

  profile_after.clear();
  profile_after.resize(Rmax, 0);

  profile_poly.clear();
  profile_poly.resize(Rmax, 0);

  profile_counter.clear();
  profile_counter.resize(Rmax, 0);


  for ( int c = 0, cn = channels.size(); c < cn; ++c ) {

    cv::dft(channels[c], channels[c], cv::DFT_COMPLEX_OUTPUT);

    cv::Mat_<std::complex<float>> spec =
        channels[c];

    double Pn = 1, Rn = 0, E = 0;
    for ( int y = 0; y < spec.rows / 2; ++y ) {
      for ( int x = 0; x < spec.cols / 2; ++x ) {

        const double R = M_SQRT1_2 *
            hyp((double) (x) / C.x,
                (double) (y) / C.y);

        if ( !coeffs.empty() ) {
          // Compute Pn(x) = a0*x + a1*x^2 + a2*x^3 + ...
          Pn = 0, Rn = 1, E = 0;
          for ( int i = 0, n = coeffs.size(); i < n; ++i ) {
            Pn += coeffs[i] * (Rn *= R);
            E += coeffs[i] * (i + 1);
          }
          // Force zero derivative at edge R=1
          Pn += 1 - E * Rn * R * R / (coeffs.size() + 2);
        }

        const int Ridx =
            (int) (R * Rmax);

        if ( Ridx < Rmax ) {
          profile_poly[Ridx] += Pn;
          profile_before[Ridx] += std::abs(spec[y][x]);
        }

        spec[y][x] *= Pn;
        spec[y][spec.cols - x - 1] *= Pn;
        spec[spec.rows - y - 1][x] *= Pn;
        spec[spec.rows - y - 1][spec.cols - x - 1] *= Pn;

        if ( Ridx < Rmax ) {
          profile_after[Ridx] += std::abs(spec[y][x]);
          profile_counter[Ridx] += 1;
        }
      }
    }

    cv::idft(channels[c], channels[c], cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
  }

  for ( int i = 0; i < Rmax; ++i ) {
    if ( profile_counter[i] > 0 ) {
      profile_before[i] = log(1 + profile_before[i] / profile_counter[i]);
      profile_after[i] = log(1 + profile_after[i] / profile_counter[i]);
      profile_poly[i] /= profile_counter[i];
    }
  }

  const int ddepth = dst.fixedType() ?
      dst.depth() :
      src.depth();

  if ( channels.size() == 1 ) {
    image = channels[0];
  }
  else {
    cv::merge(channels, image);
  }

  if ( !rc.empty() ) {
    image = image(rc);
  }

  if ( preserve_l2_norm ) {
    cv::multiply(image, saved_image_norm / cv::norm(image, cv::NORM_L2), image);
  }

  if ( image.depth() != ddepth ) {
    image.convertTo(dst, ddepth);
  }
  else if ( src.getMat().data == dst.getMat().data ) {
    image.copyTo(dst);
  }
  else {
    dst.move(image);
  }
}


void fftSharpenR1(cv::InputArray image, cv::OutputArray dst, double scale, bool preserve_l2_norm)
{
  typedef tbb::blocked_range<int> range;

  std::vector<cv::Mat2f> channels;
  double saved_image_norm = 1;
  cv::Rect rc;

  if ( preserve_l2_norm ) {
    saved_image_norm = cv::norm(image, cv::NORM_L2);
  }

  fftImageToSpectrum(image, channels, cv::Size(32, 32), &rc);

  for ( int c = 0, cn = channels.size(); c < cn; ++c ) {

    cv::Mat2f & spec = channels[c];

    fftSwapQuadrants(spec);

    if ( scale >= 0 ) {
      // sharpen

      tbb::parallel_for(range(0, spec.rows, 64),
          [&spec, scale] (const range & r) {

            const double x0 = spec.cols / 2;
            const double y0 = spec.rows / 2;

            for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
              for ( int x = 0; x < spec.cols; ++x ) {
                const double r1 = 1. + scale * hyp((x - x0) / x0, (y - y0) / y0);
                spec[y][x][0] *= r1;
                spec[y][x][1] *= r1;
              }
            }
          });
    }
    else {
      // smoothing

      tbb::parallel_for(range(0, spec.rows, 64),
          [&spec, scale] (const range & r) {

            const double x0 = spec.cols / 2;
            const double y0 = spec.rows / 2;

            for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
              for ( int x = 0; x < spec.cols; ++x ) {
                const double r1 = 1./(1. - scale * hyp((x - x0) / x0, (y - y0) / y0));
                spec[y][x][0] *= r1;
                spec[y][x][1] *= r1;
              }
            }
          });
    }

    fftSwapQuadrants(spec);
  }

  fftImageFromSpectrum(channels, dst, rc);

  if ( preserve_l2_norm ) {
    cv::multiply(dst, saved_image_norm / cv::norm(image, cv::NORM_L2), dst);
  }
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
    CF_ERROR("copyMakeFFTBorder(src=%dx%d, img, fft_size=%dx%d) fails", src.cols, src.rows, fft_size.width, fft_size.height);
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

void fftComputeAutoCorrelation(cv::InputArray src, cv::OutputArray dst, bool logscale)
{
  static const auto compute_magnitue =
      [](cv::Mat2f & src) {

        typedef std::complex<float> complex;

        const cv::Mat_<complex> spec = src;

        for ( int y = 0; y < spec.rows; ++y ) {
          for ( int x = 0; x < spec.cols; ++x ) {
            src[y][x] = std::abs(spec[y][x]);
          }
        }

      };


  cv::Mat image;
  cv::Rect rc;

  if ( !(src.cols() & 0x1) && !(src.rows() & 0x1) ) {
    image = src.getMat();
  }
  else {
    fftCopyMakeBorder(src, image, fftGetOptimalSize(src.size(), cv::Size(), nullptr, true), &rc);
  }


  std::vector<cv::Mat2f> channels;

  fftImageToSpectrum(image, channels);

  for ( int c = 0, cn = channels.size(); c < cn; ++c ) {
    compute_magnitue(channels[c]);
  }

  fftImageFromSpectrum(channels, image);

  if( true ) {
    cv::Scalar scale;
    const int cn = std::min((int) (sizeof(scale.val) / sizeof(scale.val[0])), image.channels());
    for( int c = 0; c < cn; ++c ) {
      scale.val[c] = 1./image.ptr<const float>(0)[0 + c];
    }

    cv::multiply(image, scale, image);
  }

  fftSwapQuadrants(image);

  if ( !rc.empty() ) {
    image = image(rc);
  }

  if ( logscale ) {
    // cv::log(image + cv::Scalar::all(1), image);
  }

  dst.move(image);
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

  const double cx = fftSize.width / 2.0;
  const double cy = fftSize.height / 2.0;
  const double sx = sigma_space * CV_PI * M_SQRT2 / fftSize.width;
  const double sy = sigma_space * CV_PI * M_SQRT2 / fftSize.height;

  cv::parallel_for_(cv::Range(0, fftSize.height),
      [=, &FILTER](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {
          float * __restrict dstp = FILTER[y];

          const double dy = (y - cy) * sy;
          const double dy2 = dy * dy;

          for (int x = 0; x < fftSize.width; ++x) {
            const double dx = (x - cx) * sx;
            const double dx2 = dx * dx;

            const double gaussLPF = gain * std::exp(-(dx2 + dy2));
            dstp[x] = float(inverseFilter ? gain - gaussLPF : gaussLPF);
          }
        }
      });

  if( !centerDC ) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

cv::Mat1f fftGenerateLaplacianFilter(const cv::Size & fftSize, double gain, bool squareRoot,
    bool swapQuadrants)
{
  // Isotropic Laplacian
  // The frequency step is tied to the physical dimensions of the matrix
  // fx = dx / width, fy = dy / height
  // Physical Laplacian: 4 * PI^2 * (fx^2 + fy^2)

  cv::Mat1f FILTER(fftSize);

  const double scaleX = CV_2PI / fftSize.width;
  const double scaleY = CV_2PI / fftSize.height;
  const double cx = fftSize.width / 2.0;
  const double cy = fftSize.height / 2.0;

  cv::parallel_for_(cv::Range(0, fftSize.height),
      [=, &FILTER](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {
          float * __restrict dstp = FILTER[y];

          const double dy = (y - cy) * scaleY;
          const double dy2 = dy * dy;

          for (int x = 0; x < fftSize.width; ++x) {
            const double dx = (x - cx) * scaleX;
            const double dx2 = dx * dx;

            const double dr2 = dx2 + dy2;

            dstp[x] = float(gain * (squareRoot ? sqrt(dr2) : dr2));
          }
        }
      });

  if( swapQuadrants ) {
    fftSwapQuadrants(FILTER);
  }

  return FILTER;
}

// Isotropic Butterworth: 1.0 / (1.0 + (r / rc)^(n))
cv::Mat1f fftGenerateButterworthFilter(const cv::Size & fftSize,
    double rc, int order, double gain,
    bool swapQuadrants)
{
  // The frequency step is tied to the physical dimensions of the matrix
  // fx = dx / width, fy = dy / height

  cv::Mat1f FILTER(fftSize);

  const double cx = fftSize.width / 2.0;
  const double cy = fftSize.height / 2.0;
  const double sx = CV_2PI / fftSize.width;
  const double sy = CV_2PI / fftSize.height;

  cv::parallel_for_(cv::Range(0, fftSize.height),
      [=, &FILTER](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {
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

  if( swapQuadrants ) {
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

  cv::parallel_for_(cv::Range(0, fftSize.height),
      [=, &FILTER](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {
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

  cv::parallel_for_(cv::Range(0, fftSize.height),
      [=, &FILTER](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {
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
