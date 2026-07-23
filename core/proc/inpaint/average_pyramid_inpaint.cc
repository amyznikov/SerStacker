/*
 * average_pyramid_inpaint.cc
 *
 *  Created on: Aug 28, 2021
 *      Author: amyznikov
 */

#include "average_pyramid_inpaint.h"
#include <core/proc/downstrike.h>
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/debug.h>

#define downstrike  downstrike_even
#define upject      upject_even

template<class _Tp>
static bool _average_pyramid_filter2(
    const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask,
    const cv::Mat & fallback_src, const cv::Mat1f & fallback_mask)
{
  constexpr enum cv::BorderTypes border_mode = cv::BORDER_REPLICATE;
  const cv::Size ksize(3, 3);

  cv::boxFilter(src, dst, src.type(), ksize, cv::Point(-1, -1), false, border_mode);
  cv::boxFilter(srcmask, dstmask, dstmask.type(), ksize, cv::Point(-1, -1), false, border_mode);

  parallel_for(0, dst.rows, [&](const auto & range) {
    const int cn = dst.channels();
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {

      _Tp * __restrict dstp = dst.ptr<_Tp>(y);
      float * __restrict mskp = dstmask.ptr<float>(y);

      const _Tp* __restrict fbk_src = fallback_src.ptr<_Tp>(y);
      const float* __restrict fbk_msk = fallback_mask.ptr<float>(y);

      for ( int x = 0; x < dst.cols; ++x, ++mskp, ++fbk_msk, dstp += cn, fbk_src += cn ) {
        if ( *fbk_msk ) {
          *mskp = 1;
          for ( int c = 0; c < cn; ++c ) {
            dstp[c] = fbk_src[c];
          }
        }
        else if ( *mskp ) {
          const float scale = 1.0f / *mskp;
          *mskp = 1;
          for ( int c = 0; c < cn; ++c ) {
            dstp[c] = cv::saturate_cast<_Tp>(dstp[c] * scale);
          }
        }
      }
    }
  });

  return true;
}

static bool average_pyramid_filter2(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask,
    const cv::Mat & fallback_src, const cv::Mat1f & fallback_mask)
{
  CV_DISPATCH(src.depth(), _average_pyramid_filter2, src, srcmask, dst, dstmask, fallback_src, fallback_mask);
  CF_ERROR("Invalid src.depth()=%d", src.depth());
  return false;
}

static void average_pyramid_recurse(cv::Mat & image, cv::Mat1f & mask, int max_levels)
{
  if ( std::min(image.cols, image.rows) > 1 && max_levels > 0 ) {

    INSTRUMENT_REGION("");

    cv::Mat filtered_image;
    cv::Mat1f filtered_mask;

    average_pyramid_filter2(image, mask, filtered_image, filtered_mask, image, mask);
    downstrike(filtered_image, filtered_image);
    downstrike(filtered_mask, filtered_mask);

    if ( cv::countNonZero(filtered_mask) < filtered_mask.size().area() ) {
      average_pyramid_recurse(filtered_image, filtered_mask, max_levels - 1);
    }

    upject(filtered_image, filtered_image, image.size(), &filtered_mask);
    average_pyramid_filter2(filtered_image, filtered_mask,
        filtered_image, filtered_mask,
        image, mask);

    image = std::move(filtered_image);
    mask = std::move(filtered_mask);
  }
}

void average_pyramid_inpaint(cv::InputArray _src, cv::InputArray _mask,
    cv::OutputArray dst, cv::OutputArray _dstmask, int max_levels)
{
  INSTRUMENT_REGION("");

  const cv::Mat1b mask = _mask.getMat();

  if ( mask.empty() || cv::countNonZero(mask) == mask.size().area() ) {
    _src.copyTo(dst);
    if (_dstmask.needed()) {
      mask.copyTo(_dstmask);
    }
    return;
  }

  cv::Mat src;
  cv::Mat1f msk;

  _src.getMat().copyTo(src, mask);
  mask.convertTo(msk, CV_32F, 1.0 / 255.0);

  average_pyramid_recurse(src, msk, max_levels);

  const int ddepth = dst.fixedType() ? dst.type() : _src.type();

  src.convertTo(dst, ddepth);

  if (_dstmask.needed()) {
    msk.convertTo(_dstmask, CV_8U, 255.0);
  }
}

