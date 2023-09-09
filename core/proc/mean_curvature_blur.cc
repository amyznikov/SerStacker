/*
 * gegl_mean_curvature_blur.cc
 *
 *  Created on: Aug 1, 2022
 *      Author: amyznikov
 *
 * Derived from gegl/operations/common/mean-curvature-blur.c
 */

#include "mean_curvature_blur.h"
#if HAVE_TBB
# include <tbb/tbb.h>
#endif

static inline float pow3(float x)
{
  return x * x * x;
}

static void mean_curvature_flow(const cv::Mat & src_image, cv::Mat & dst_image)
{
#define CENTER(p)  p[(y) * stride + (x) * cn + (c)]
#define LEFT(p)  p[(y) * stride + ((x)-1) * cn + (c)]
#define RIGHT(p)  p[(y) * stride + ((x)+1) * cn + (c)]
#define TOP(p)  p[((y)-1) * stride + (x) * cn + (c)]
#define BOTTOM(p)  p[((y)+1) * stride + (x) * cn + (c)]
#define TOPLEFT(p)  p[((y)-1) * stride + ((x)-1) * cn + (c)]
#define TOPRIGHT(p)  p[((y)-1) * stride + ((x)+1) * cn + (c)]
#define BOTTOMLEFT(p)  p[((y)+1) * stride + ((x)-1) * cn + (c)]
#define BOTTOMRIGHT(p)  p[((y)+1) * stride + ((x)+1) * cn + (c)]
//#define POW2(a) ((a)*(a))

  const int cn = src_image.channels();
  const int stride = src_image.cols * cn;
  const int rows = src_image.rows;
  const int cols = src_image.cols;
  const float *src = (const float*) src_image.data;
  float *dst = (float*) dst_image.data;

#if HAVE_TBB
  typedef tbb::blocked_range<int> tbb_range;
  tbb::parallel_for(tbb_range(1, rows - 1, 256),
      [=](const tbb_range & range) {
        for( int y = range.begin(); y < range.end(); ++y ) {
#else
          for( int y = 1; y < rows - 1; ++y ) {
#endif

          for( int x = 1; x < cols - 1; ++x ) {

            for( int c = 0; c < cn; ++c ) {

              CENTER(dst) = CENTER(src);

              const float dx = RIGHT(src) - LEFT(src);
              const float dy = BOTTOM(src) - TOP(src);
              const float magnitude = std::sqrt(dx * dx + dy * dy);

              if( magnitude > 10 * FLT_EPSILON ) {

                const float dx2 = dx * dx;
                const float dy2 = dy * dy;
                const float d = std::sqrt(pow3(dx2 + dy2)) ;

                if ( d > 10 * FLT_EPSILON ) {

                  const float dxx = RIGHT(src) + LEFT(src) - 2 * CENTER(src);
                  const float dyy = BOTTOM(src) + TOP(src) - 2 * CENTER(src);
                  const float dxy = (BOTTOMRIGHT(src) - TOPRIGHT(src) - BOTTOMLEFT(src) + TOPLEFT(src)) / 4;

                  const float n = dx2 * dyy + dy2 * dxx - 2. * dx * dy * dxy;
                  const float mean_curvature = n / d;

                  CENTER(dst) += magnitude * mean_curvature / 4;
                }
              }
            }
          }
        }
#if HAVE_TBB
        });
#endif

#undef CENTER
#undef LEFT
#undef RIGHT
#undef TOP
#undef BOTTOM
#undef TOPLEFT
#undef TOPRIGHT
#undef BOTTOMLEFT
#undef BOTTOMRIGHT
}

void mean_curvature_blur(cv::InputArray src, cv::OutputArray dst, int iterations)
{
  cv::Mat src_buf, dst_buf;

  const int src_rows = src.rows();
  const int src_cols = src.cols();
  const int cn = src.channels();

  if( src.depth() == CV_32F ) {
    cv::copyMakeBorder(src, src_buf, 1, 1, 1, 1, cv::BORDER_REPLICATE);
  }
  else {
    src.getMat().convertTo(src_buf, CV_32F);
    cv::copyMakeBorder(src_buf, src_buf, 1, 1, 1, 1, cv::BORDER_REPLICATE);
  }

  //dst_buf.create(src_buf.size(), src_buf.type());
  src_buf.copyTo(dst_buf);

  for( int iteration = 0; iteration < iterations; ++iteration ) {
    mean_curvature_flow(src_buf, dst_buf);
    cv::swap(src_buf, dst_buf);
  }

  src_buf(cv::Rect(1, 1, src_cols, src_rows)).convertTo(dst, src.depth());
}
