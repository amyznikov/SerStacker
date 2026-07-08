/*
 * adaptive_gaussian_blur.cc
 *
 *  Created on: Jul 8, 2026
 *      Author: amyznikov
 */
#include "adaptive_gaussian_blur.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_TYPE>()
{
  static const c_enum_member members[] = {
      { ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_FILTERED, "FILTERED", "" },
      { ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_BLUR1, "BLUR1", "" },
      { ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_BLUR2, "BLUR2", "" },
      { ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_LPG, "LPG", "" },
      { ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_FILTERED}
  };
  return members;
}

static void compute_gradient(const cv::Mat & src, cv::Mat & g)
{
  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 1, 0, 7, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  cv::Mat gx, gy;
  cv::sepFilter2D(src, gx, CV_32F, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, gy, CV_32F, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::magnitude(gx, gy, g);
}

static void compute_modlaplace(const cv::Mat & src, cv::Mat & l, double scale)
{
  static thread_local cv::Mat Kxx, Kyy;
  if( Kxx.empty() ) {
    cv::getDerivKernels(Kxx, Kyy, 2, 0, 7, true, CV_32F);
    Kxx *= M_SQRT2;
    Kyy *= M_SQRT2;
  }

  cv::Mat lx, ly;

  cv::sepFilter2D(src, lx, CV_32F, Kxx * scale, Kyy, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, ly, CV_32F, Kyy, Kxx * scale, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::magnitude(lx, ly, l);
}

static void compute_lpg(cv::InputArray _src, cv::OutputArray _dst, double gsigma, double lpgk)
{
  cv::Mat src, g, l;

  cv::GaussianBlur(_src, src, cv::Size(), gsigma, gsigma);
  compute_gradient(src, g);
  cv::GaussianBlur(g, g, cv::Size(), gsigma, gsigma);

  if ( !(lpgk > 0) ) {
    _dst.move(g);
  }
  else {
    compute_modlaplace(src, l, lpgk);
    cv::add(l, g, _dst);
  }
}

void adaptive_gaussian_blur(cv::InputArray _src, cv::OutputArray _dst,
    double sigma_hpass, double sigma_lpass, double lpg_scale, double lpgk,
    ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_TYPE outputDisplay)
{
  const int cn = _src.channels();

  cv::Mat src, gray, blur1, blur2, lpg, dst;

  if ( _src.depth() == CV_32F ) {
    src = _src.getMat();
  }
  else {
    _src.getMat().convertTo(src, CV_32F);
  }

  if( cn == 1 ) {
    gray = src;
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  if( sigma_hpass > 0 ) {
    cv::GaussianBlur(src, blur1, cv::Size(), sigma_hpass, sigma_hpass);
  }
  else {
    blur1 = src;
  }

  if( outputDisplay == ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_BLUR1 ) {
    _dst.move(blur1);
    return;
  }

  if( sigma_lpass > 0 ) {
    cv::GaussianBlur(blur1, blur2, cv::Size(), sigma_lpass, sigma_lpass);
  }
  else {
    blur2 = blur1;
  }

  if( outputDisplay == ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_BLUR2 ) {
    _dst.move(blur2);
    return;
  }

  const double sigma_se = 1.5;
  compute_lpg(gray, lpg, sigma_se, lpgk);
  if( outputDisplay == ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_LPG) {
    _dst.move(lpg);
    return;
  }

  const cv::Size size = _src.size();
  dst.create(size, src.type());

  cv::parallel_for_(cv::Range(0, size.height),
      [&, size, cn, lpg_scale](const auto & range) {
        for ( int y = range.start; y < range.end; ++y ) {
          const float * lpgp = lpg.ptr<const float>(y);
          const float * blur1p = blur1.ptr<const float>(y);
          const float * blur2p = blur2.ptr<const float>(y);
          float * __restrict dstp = dst.ptr<float>(y);

          for ( int x = 0; x < size.width; ++x ) {
            const float w = lpgp[x] * lpg_scale;
            const float K = 1.f / ( w + 1.f );
            for ( int c = 0; c < cn; ++c ) {
              dstp[x * cn + c] = (blur1p[x * cn + c] * w + blur2p[x * cn + c]) * K;
            }
          }
        }
      });

//  cv::Mat K;
//  if( lpg_scale > 0 ) {
//    cv::multiply(lpg, lpg_scale, lpg);
//  }
//
//  cv::divide(1.0, lpg + 1.0, K);
//
//  if( cn > 1 ) {
//    cv::cvtColor(lpg, lpg, cv::COLOR_GRAY2BGR);
//    cv::cvtColor(K, K, cv::COLOR_GRAY2BGR);
//  }
//
//  cv::multiply(lpg.mul(blur1) + blur2, K, _dst);

  const int ddepth = _dst.fixedType() ? _dst.depth() : _src.depth();
  if ( ddepth == dst.depth() ) {
    _dst.move(dst);
  }
  else {
    dst.convertTo(_dst, ddepth);
  }
}
