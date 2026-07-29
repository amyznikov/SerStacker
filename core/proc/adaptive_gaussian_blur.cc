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

static void compute_gradient_map(cv::InputArray _src, cv::OutputArray _dst, int gradius)
{
  cv::Mat g;

  const int mksize = 2 * gradius + 1;
  const int gksize = 2 * gradius + 3;

  const cv::Mat SE =
      cv::getStructuringElement(mksize < 5 ? cv::MORPH_RECT : cv::MORPH_ELLIPSE,
          cv::Size(mksize, mksize));

  cv::morphologyEx(_src, g, cv::MORPH_GRADIENT, SE,
      cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

  cv::GaussianBlur(g, _dst, cv::Size(gksize, gksize), 0, 0,
      cv::BORDER_DEFAULT);

  const double m = cv::mean(_dst)[0];
  cv::multiply(_dst, _dst, _dst, 0.1 / (m * m));
}

void adaptive_gaussian_blur(cv::InputArray _src, cv::OutputArray _dst,
    double sigma_hpass, double sigma_lpass, double kscale, int kradius,
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

  compute_gradient_map(gray, lpg, kradius);
  if( outputDisplay == ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_LPG) {
    _dst.move(lpg);
    return;
  }

  const cv::Size size = _src.size();
  dst.create(size, src.type());

  cv::parallel_for_(cv::Range(0, size.height),
      [&, size, cn, kscale](const auto & range) {
        for ( int y = range.start; y < range.end; ++y ) {
          const float * lpgp = lpg.ptr<const float>(y);
          const float * blur1p = blur1.ptr<const float>(y);
          const float * blur2p = blur2.ptr<const float>(y);
          float * __restrict dstp = dst.ptr<float>(y);

          for ( int x = 0; x < size.width; ++x ) {
            const float w = lpgp[x] * kscale;
            const float K = 1.f / (w + 1.f);
            for ( int c = 0; c < cn; ++c ) {
              dstp[x * cn + c] = (blur1p[x * cn + c] * w + blur2p[x * cn + c]) * K;
            }
          }
        }
      });

  const int ddepth = _dst.fixedType() ? _dst.depth() : _src.depth();
  if ( ddepth == dst.depth() ) {
    _dst.move(dst);
  }
  else {
    dst.convertTo(_dst, ddepth);
  }
}
