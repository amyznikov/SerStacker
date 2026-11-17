/*
 * c_radon_transfrom_routine.cc
 *
 *  Created on: Oct 10, 2026
 *      Author: amyznikov
 */

#include "c_radon_transfrom_routine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/radon_transform.hpp>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/proc/divide.h>
#include <core/proc/fft.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<c_radon_transfrom_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_radon_transfrom_routine::DISPLAY_SINOGRAM, "SINOGRAM", },
      { c_radon_transfrom_routine::DISPLAY_P_SPECTRUM, "P_SPECTRUM" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_CART, "CMLPX_SPEC_CART" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_CART_POLAR, "CMLPX_SPEC_CART_POLAR" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_POLAR, "CMLPX_SPEC_POLAR" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_POLAR_POLAR, "CMLPX_SPEC_POLAR_POLAR" },
      { c_radon_transfrom_routine::DISPLAY_SINOGRAM, },
  };
  return members;
}

static void ifftshift(cv::InputArray _src, cv::OutputArray _dst)
{
  cv::Mat src = _src.getMat();
  CV_Assert(src.dims == 2);

  const int cx = (src.cols + 1) / 2;
  const int cy = (src.rows + 1) / 2;

  if( cx == 0 || cy == 0 ) {
    src.copyTo(_dst);
    return;
  }

  cv::Mat q0(src, cv::Rect(0, 0, cx, cy));
  cv::Mat q1(src, cv::Rect(cx, 0, src.cols - cx, cy));
  cv::Mat q2(src, cv::Rect(0, cy, cx, src.rows - cy));
  cv::Mat q3(src, cv::Rect(cx, cy, src.cols - cx, src.rows - cy));

  cv::Mat dst;
  if( src.data == _dst.getMat().data ) {
    dst.create(src.size(), src.type());
  }
  else {
    _dst.create(src.size(), src.type());
    dst = _dst.getMat();
  }

  const int ncx = src.cols - cx;
  const int ncy = src.rows - cy;

  // q0 <-> q3, q1 <-> q2
  q3.copyTo(dst(cv::Rect(0, 0, ncx, ncy)));
  q2.copyTo(dst(cv::Rect(ncx, 0, cx, ncy)));
  q1.copyTo(dst(cv::Rect(0, ncy, ncx, cy)));
  q0.copyTo(dst(cv::Rect(ncx, ncy, cx, cy)));

  if( dst.data != _dst.getMat().data ) {
    dst.copyTo(_dst);
  }
}

static void _shift1D_rows(cv::InputArray _src, cv::OutputArray _dst, int cx)
{
  cv::Mat src = _src.getMat();
  CV_Assert(src.dims == 2);

  const int rows = src.rows;
  const int cols = src.cols;

  if( cx <= 0 || cx >= cols ) {
    src.copyTo(_dst);
    return;
  }

  const int ncx = cols - cx;
  const size_t elemSize = src.elemSize();
  const size_t leftBytes = size_t(cx) * elemSize;
  const size_t rightBytes = size_t(ncx) * elemSize;

  cv::Mat dst = createOutOfPlace(_src, _dst, _src.size(), _src.type());

  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range), yend = rend(range); y < yend; ++y) {
      const uint8_t * const srcp = src_base + y * src_stride;
      uint8_t * const dstp = dst_base + y * dst_stride;
      std::memcpy(dstp, srcp + leftBytes, rightBytes);
      std::memcpy(dstp + rightBytes, srcp, leftBytes);
    }
  });

  assignOutOfPlace(_dst, dst);
}

static void ifftshift1D_rows(cv::InputArray src, cv::OutputArray dst)
{
    _shift1D_rows(src, dst, (src.cols() + 1) / 2);
}

static void fftshift1D_rows(cv::InputArray src, cv::OutputArray dst)
{
  _shift1D_rows(src, dst, src.cols() / 2);
}

static void complexSpect2polar(
    const cv::Mat2f & cmplxSpecCart,
    cv::Mat2f & outCmplxSpecPolar,
    int numAngles = 180,
    int rBinSize = 2)
{
  CV_Assert(rBinSize >= 1);

  const cv::Size fftSize = cmplxSpecCart.size();
  const int cx = fftSize.width / 2;
  const int cy = fftSize.height / 2;

  const double rmax = 0.5 * std::sqrt(double(fftSize.width) * fftSize.width + double(fftSize.height) * fftSize.height);

  const int halfBins = cvCeil(rmax / rBinSize);
  const int nRadii = 2 * halfBins + 1;
  const int rCenter = halfBins;
  const double rscale = (rCenter - 1) / rmax; // [0, nRadii - 1]
  const double ascale = double(numAngles) / CV_PI;

  const cv::Size polarSize(nRadii, numAngles);
  cv::Mat1f cntr(polarSize, 0.f);

  outCmplxSpecPolar.create(polarSize);
  outCmplxSpecPolar.setTo(cv::Scalar::all(0));

  const uint8_t * const src_base = cmplxSpecCart.ptr();
  const size_t src_stride = cmplxSpecCart.step;

  uint8_t * const out_base = outCmplxSpecPolar.ptr();
  const size_t out_stride = outCmplxSpecPolar.step;

  uint8_t * const cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  for( int y = 0; y <= cy; ++y ) {
    const cv::Vec2f * srcp = (const cv::Vec2f*) (src_base + y * src_stride);

    const int xbeg = (y < cy) ? 0 : cx;
    for( int x = xbeg; x < fftSize.width; ++x ) {
      const int dx = x - cx;
      const int dy = y - cy;

      double theta = std::atan2(double(-dy), double(dx));
      if( theta < 0.0 ) {
        theta += CV_PI;
      }

      int aIdx = int(theta * ascale + 0.5);
      if( aIdx >= numAngles ) {
        aIdx = 0;
      }

      const double r = std::sqrt(double(dx) * dx + double(dy) * dy);
      const int rOffset = int(r * rscale + 0.5);

      const int sign = (y == cy && x < cx) ? -1 : 1;
      const int rIdxPos = rCenter + sign * rOffset;
      const int rIdxNeg = rCenter - sign * rOffset;

      const cv::Vec2f val = srcp[x];
      const cv::Vec2f valConj(val[0], -val[1]);

      cv::Vec2f * outRow = (cv::Vec2f*) (out_base + aIdx * out_stride);
      float * cntrRow = (float*) (cntr_base + aIdx * cntr_stride);

      outRow[rIdxPos] += val;
      cntrRow[rIdxPos] += 1.f;

      outRow[rIdxNeg] += valConj;
      cntrRow[rIdxNeg] += 1.f;
    }
  }

  // compute average, DONT GENERATE THIS FUNCTION - IT ALREADY EXISTS!!!!!
  divideImages(outCmplxSpecPolar, cntr, outCmplxSpecPolar, -1, 1, 0.5);
}

/**
* @brief Generate transformation map for cv::remap (Cartesian -> Polar).
*   The radial dimension of the output matrix is ​​strictly tied to the physical FFT size to eliminate distortions.
*
* @param outputMap Output two-channel CV_32FC2 matrix for cv::remap.
* @param fftSize Size of the initial Cartesian spectrum (must be even).
* @param numAngles Number of discrete scanning angles (height of the output matrix).
* */
static void createCart2PolarRemap(cv::Mat2f & outputMap, const cv::Size & fftSize, const int numAngles)
{
  const float centerX = fftSize.width / 2;
  const float centerY = fftSize.height / 2;

  // The number of radial samples is strictly the distance from the center to the nearest frame edge.
  // Since fftSize is even, this is simply lossless integer division.
  const int numRadii = (fftSize.width < fftSize.height) ? (fftSize.width / 2) : (fftSize.height / 2);

  // The radial step equals 1 pixel of the source matrix.
  // The rIdx multiplier can be used directly.
  const float stepTheta = float(2.0 * CV_PI / numAngles);

  // The matrix is ​​created with the exact size physically justified by the FFT spectrum.
  outputMap.create(cv::Size(numRadii, numAngles));

  uint8_t * const outmap_base = outputMap.ptr();
  const size_t outmap_stride = outputMap.step;

  parallel_for(0, numAngles, [=](const auto & range) {
    for( int thetaIdx = rbegin(range); thetaIdx < rend(range); ++thetaIdx ) {
      cv::Vec2f * __restrict mp = (cv::Vec2f * ) (outmap_base + thetaIdx * outmap_stride);
      const float theta = float(thetaIdx) * stepTheta;
      const float cos_t = std::cos(theta);
      const float sin_t = std::sin(theta);

      // normalization factor for the square FFT frequency grid.
      // stretches the beam step along the diagonals (where max is 0.707, turning the step into 1.414)
      const float grid_scale = 1.0f / std::fmax(std::abs(cos_t), std::abs(sin_t));
      for( int rIdx = 0; rIdx < numRadii; ++rIdx ) {
        const float r = float(rIdx* grid_scale);
        mp[rIdx] = cv::Vec2f(centerX + r * cos_t, centerY + r * sin_t);
      }
    }
  });
}

bool c_radon_transfrom_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, theta);
    SERIALIZE_OPTION(settings, save, *this, start_angle);
    SERIALIZE_OPTION(settings, save, *this, end_angle);
    SERIALIZE_OPTION(settings, save, *this, crop);
    SERIALIZE_OPTION(settings, save, *this, norm);
    SERIALIZE_OPTION(settings, save, *this, _display);
    return true;
  }
  return false;
}

void c_radon_transfrom_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display), "");
//  ctlbind(ctls, "theta", ctx(&this_class::theta), "");
//  ctlbind(ctls, "start_angle", ctx(&this_class::start_angle), "");
//  ctlbind(ctls, "end_angle", ctx(&this_class::end_angle), "");
//  ctlbind(ctls, "crop", ctx(&this_class::crop), "");
//  ctlbind(ctls, "norm", ctx(&this_class::norm), "");
}

bool c_radon_transfrom_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat2f P_SPECTRUM;
  cv::Mat2f P_SPECTRUM_POLAR;
  cv::Mat1f SINOGRAM;

  const cv::Size fftSize = image.size();

  ifftshift(image, image);
  cv::dft(image, P_SPECTRUM, cv::DFT_COMPLEX_OUTPUT);
  fftSwapQuadrants(P_SPECTRUM);
  complexSpect2polar(P_SPECTRUM, P_SPECTRUM_POLAR, 360, 4);
  ifftshift1D_rows(P_SPECTRUM_POLAR, P_SPECTRUM_POLAR);
//  fftSpectrumToPolar(P_SPECTRUM_POLAR, image);
  cv::dft(P_SPECTRUM_POLAR, SINOGRAM, cv::DFT_INVERSE | cv::DFT_ROWS | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
  fftshift1D_rows(SINOGRAM, image);
  // //image.assign(SINOGRAM);

  mask.release();

  return true;
}

