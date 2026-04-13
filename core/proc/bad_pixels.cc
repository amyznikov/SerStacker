/*
 * bad_pixels.cc
 *
 *  Created on: May 13, 2023
 *      Author: amyznikov
 */

#include "bad_pixels.h"
#include <core/proc/run-loop.h>
#include <core/proc/morphology.h>
#include <core/debug.h>


static void computeVariationImages(const cv::Mat & src,
    cv::Mat & median, cv::Mat & var, cv::Mat & mvar,
    double k, int mksize)
{
  const double minimal_mean_variation_for_very_smooth_images =
      src.depth() < CV_32F ? 1 : 1e-3;

  const double k8 = k / 8.0;

  cv::medianBlur(src, median, mksize);
  cv::absdiff(src, median, var);
  cv::boxFilter(var, mvar, -1, cv::Size(3, 3));
  cv::addWeighted(mvar, k8 * 9.0, var, -k8, minimal_mean_variation_for_very_smooth_images, mvar);
}

/**
 * bayer_image: Must be single-channel bayer pattern image.
 */
template<class T>
static void _bayer_denoise(cv::Mat & bayer_image, double k)
{
  if( (bayer_image.cols & 0x1) || (bayer_image.rows & 0x1) || bayer_image.channels() != 1 ) {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        bayer_image.cols, bayer_image.rows, bayer_image.channels());
    return;
  }

  cv::Mat_<T> src = bayer_image;

  using Vec4T = cv::Vec<T, 4>;
  using Mat4T = cv::Mat_<Vec4T>;

  Mat4T bayer_planes, median, var, mvar;

  /* Create 4-pane image with separate panes for bayer colors */
  bayer_planes.create(src.rows / 2, src.cols / 2);

  parallel_for(0, src.rows / 2, [&, xmax = src.cols/2](const auto & r) {
    for( int y = rbegin(r); y < rend(r); ++y ) {

      const T * srcp0 = src[2 * y + 0];
      const T * srcp1 = src[2 * y + 1];
      Vec4T * dstp = bayer_planes[y];

      for( int x = 0; x < xmax; ++x ) {
        dstp[x][0] = srcp0[2 * x + 0];
        dstp[x][1] = srcp0[2 * x + 1];
        dstp[x][2] = srcp1[2 * x + 0];
        dstp[x][3] = srcp1[2 * x + 1];
      }
    }
  });

  /* compute local variation maps */
  computeVariationImages(bayer_planes, median, var, mvar, k, 5);

  /* apply local variation threshold */
  parallel_for(0, src.rows / 2, [&, xmax = src.cols / 2](const auto & r) {
    for( int y = rbegin(r); y < rend(r); ++y ) {

      const auto * mp = median[y];
      const auto * vp = var[y];
      const auto * mvp = mvar[y];

      T * src0 = src[2 * y + 0];
      T * src1 = src[2 * y + 1];

      for( int x = 0; x < xmax; ++x ) {
        if( vp[x][0] > mvp[x][0] ) {
          src0[2 * x + 0] = mp[x][0];
        }
        if( vp[x][1] > mvp[x][1] ) {
          src0[2 * x + 1] = mp[x][1];
        }
        if( vp[x][2] > mvp[x][2] ) {
          src1[2 * x + 0] = mp[x][2];
        }
        if( vp[x][3] > mvp[x][3] ) {
          src1[2 * x + 1] = mp[x][3];
        }
      }
    }
  });
}

void median_filter_bad_pixels(cv::Mat & image, double variation_threshold, bool is_bayer_pattern)
{
  INSTRUMENT_REGION("");

  if( !is_bayer_pattern ) {
    cv::Mat median, var, mvar;
    computeVariationImages(image, median, var, mvar, variation_threshold, 5);
    median.copyTo(image, var > variation_threshold * mvar);
  }
  else {
    switch (image.depth()) {
      case CV_8U:
        return _bayer_denoise<uint8_t>(image, variation_threshold);
      case CV_8S:
        return _bayer_denoise<int8_t>(image, variation_threshold);
      case CV_16U:
        return _bayer_denoise<uint16_t>(image, variation_threshold);
      case CV_16S:
        return _bayer_denoise<int16_t>(image, variation_threshold);
      case CV_32S:
        return _bayer_denoise<int32_t>(image, variation_threshold);
      case CV_32F:
        return _bayer_denoise<float>(image, variation_threshold);
      case CV_64F:
        return _bayer_denoise<double>(image, variation_threshold);
    }
  }
}
