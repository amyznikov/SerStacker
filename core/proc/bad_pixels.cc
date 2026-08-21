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

template<class _Tp>
static bool _median_denoise(cv::Mat & image, double _k)
{
  const int rows = image.rows;
  const int cols = image.cols;
  const int channels = image.channels();
  const int ksize = 5;

  cv::Mat median, mad;
  cv::medianBlur(image, median, ksize);
  cv::absdiff(image, median, mad);
  cv::boxFilter(mad, mad, -1, cv::Size(ksize, ksize), cv::Point(ksize / 2, ksize / 2), true, cv::BORDER_DEFAULT);

  uint8_t * const image_base = (uint8_t*) image.ptr();
  const size_t image_stride = image.step;

  const uint8_t * const median_base = median.ptr();
  const size_t median_stride = median.step;

  const uint8_t * const mad_base = (const uint8_t*) mad.ptr();
  const size_t mad_stride = mad.step;

  const float mv = image.depth() < CV_32F ? 1.f : 1.f / 256.f;

  parallel_for(0, rows, [=](const auto & range) {
    const float k = _k;
    for( int y = rbegin(range); y < rend(range); ++y ) {

      _Tp * __restrict img = (_Tp *) (image_base + y * image_stride);
      const _Tp * med = (const _Tp*)(median_base + y * median_stride);
      const _Tp * mad = (const _Tp*)(mad_base + y * mad_stride);

      for( int x = 0; x < cols; ++x, img += channels, med += channels, mad += channels ) {
        for( int c = 0; c < channels; ++c ) {
          const float p = img[c];
          const float m = med[c];
          if ( std::abs(m - p) > k * mad[c] + mv ) {
            img[c] = m;
          }
        }
      }
    }
  });

  return true;
}

bool median_filter_bad_pixels(cv::Mat & image, double variation_threshold, COLORID color_id)
{
  INSTRUMENT_REGION("");

  if( is_bayer_pattern(color_id) ) {
    return bayer_denoise(image, variation_threshold, color_id, false);
  }

  CV_DISPATCH(image.depth(), _median_denoise, image, variation_threshold);
  CF_ERROR("Not supported image.depth()=%d ", image.depth());

  return false;
}
