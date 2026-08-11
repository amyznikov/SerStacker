/*
 * bad_pixels.cc
 *
 *  Created on: May 13, 2023
 *      Author: amyznikov
 */

#include "bad_pixels.h"
#include <core/proc/run-loop.h>
#include <core/proc/morphology.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>

/**
 * bayer_image: Must be single-channel bayer pattern image.
 */
template<class T>
static bool _bayer_denoise(cv::Mat & bayer_image, double _k, COLORID color_id)
{
  INSTRUMENT_REGION("");

  if( (bayer_image.cols & 0x1) || (bayer_image.rows & 0x1) || bayer_image.channels() != 1 ) {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        bayer_image.cols, bayer_image.rows, bayer_image.channels());
    return false;
  }

  cv::Mat_<T> src = bayer_image;
  const size_t bayer_step = bayer_image.step / sizeof(T);

  using Vec4T = cv::Vec<T, 4>;
  cv::Mat_<Vec4T> planes, median, mad;

  const int rows4 = src.rows / 2;
  const int cols4 = src.cols / 2;

  planes.create(rows4, cols4);

  parallel_for(0, rows4, [=, &src, &planes](const auto & range) {
    for( int y = rbegin(range); y < rend(range); ++y ) {
      const T * __restrict src0 = src[2 * y + 0];
      const T * __restrict src1 = src[2 * y + 1];
      T * __restrict dstp = (T * )(planes[y]);
      for( int x = 0; x < cols4; ++x, src0 += 2, src1 += 2, dstp += 4 ) {
        dstp[0] = src0[0];
        dstp[1] = src0[1];
        dstp[2] = src1[0];
        dstp[3] = src1[1];
      }
    }
  });

  cv::medianBlur(planes, median, 3);
  cv::absdiff(planes, median, mad);
  cv::boxFilter(mad, mad, -1, cv::Size(3, 3), cv::Point(1, 1), true, cv::BORDER_REPLICATE);

  const Vec4T * const planes_base = (const Vec4T*) planes.ptr();
  const Vec4T * const median_base = (const Vec4T*) median.ptr();
  const Vec4T * const mad_base = (const Vec4T*) mad.ptr();
  T * const bayer_base = bayer_image.ptr<T>();

  const size_t median_step = median.step / sizeof(Vec4T);
  const size_t planes_step = planes.step / sizeof(Vec4T);
  const size_t mad_step = mad.step / sizeof(Vec4T);

  const float minvar = src.depth() < CV_32F ? 1.f : 1.f / 256.f;

  parallel_for(0, rows4, [=](const auto & range) {
    const float k = _k;
    for( int y = rbegin(range); y < rend(range); ++y ) {

      const T * __restrict plane = (const T * )(planes_base + y * planes_step);
      const T * __restrict median = (const T * )(median_base + y * median_step);
      const T * __restrict mad = (const T * )(mad_base + y * mad_step);
      T * __restrict bayer0 = bayer_base + (2 * y + 0) * bayer_step;
      T * __restrict bayer1 = bayer_base + (2 * y + 1) * bayer_step;

      for( int x = 0; x < cols4; ++x, plane += 4, median += 4, mad += 4 ) {
        const float m0 = median[0], m1 = median[1], m2 = median[2], m3 = median[3];
        if ( std::abs(m0 - plane[0]) > k * mad[0] + minvar ) {
          bayer0[2 * x + 0] = m0;
        }
        if (std::abs(m1 - plane[1]) > k * mad[1] + minvar ) {
          bayer0[2 * x + 1] = m1;
        }
        if (std::abs(m2 - plane[2]) > k * mad[2] + minvar ) {
          bayer1[2 * x+ 0] = m2;
        }
        if (std::abs(m3 - plane[3]) > k * mad[3] + minvar ) {
          bayer1[2 * x + 1] = m3;
        }
      }
    }
  });

  return true;
}


static void computeVariationImages(const cv::Mat & src,
    cv::Mat & median, cv::Mat & var, cv::Mat & mvar,
    double k, int mksize)
{
  INSTRUMENT_REGION("");

  const double minimal_mean_variation_for_very_smooth_images =
      src.depth() < CV_32F ? 1 : 1e-3;

  const double k8 = k / 8.0;

  cv::medianBlur(src, median, mksize);
  cv::absdiff(src, median, var);
  cv::boxFilter(var, mvar, -1, cv::Size(3, 3));
  cv::addWeighted(mvar, k8 * 9.0, var, -k8, minimal_mean_variation_for_very_smooth_images, mvar);
}

bool median_filter_bad_pixels(cv::Mat & image, double variation_threshold, COLORID color_id)
{
  INSTRUMENT_REGION("");

  if( is_bayer_pattern(color_id) ) {
    CV_DISPATCH(image.depth(), _bayer_denoise, image, variation_threshold, color_id);
    return false;
  }

  if ( true ) {
    cv::Mat median, var, mvar;
    computeVariationImages(image, median, var, mvar, variation_threshold, 5);
    median.copyTo(image, var > variation_threshold * mvar);
  }

  return true;
}
