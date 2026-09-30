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
static bool _average_pyramid_filter_down(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask)
{
  INSTRUMENT_REGION("");

  constexpr cv::BorderTypes border_mode = cv::BORDER_REPLICATE;
  const cv::Size ksize(3, 3);

  if ( true ) {
    INSTRUMENT_REGION("boxFilter");
    parallel_invoke(
        [&] { cv::boxFilter(src, dst, src.depth(), ksize, cv::Point(-1, -1), false, border_mode); },
        [&] { cv::boxFilter(srcmask, dstmask, dstmask.depth(), ksize, cv::Point(-1, -1), false, border_mode); }
    );
  }

  if ( true ) {
    INSTRUMENT_REGION("main_loop");

    const cv::Size dstSize = dst.size();
    const int cn = dst.channels();

    uint8_t * dst_base = dst.ptr();
    const size_t dst_stride = dst.step;
    uint8_t * dstmask_base = dstmask.ptr();
    const size_t dstmask_stride = dstmask.step;

    parallel_for(0, dstSize.height, [=](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {

        _Tp * __restrict dstp = (_Tp * ) (dst_base + y * dst_stride);
        float * __restrict mskp = (float * ) (dstmask_base + y * dstmask_stride);

        for ( int x = 0; x < dstSize.width; ++x, ++mskp ) {
          const float m = *mskp;
          const float scale = m ? 1.0f / m : 0.f;
          *mskp = m ? 1.0f : 0.f;
          for ( int c = 0; c < cn; ++c, ++dstp ) {
            *dstp = m ? cv::saturate_cast<_Tp>(*dstp * scale) : _Tp(0);
          }
        }
      }
    });
  }
  return true;
}

static bool average_pyramid_filter_down(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask)
{
  CV_DISPATCH(src.depth(), _average_pyramid_filter_down, src, srcmask, dst, dstmask);
  CF_ERROR("Invalid src.depth()=%d", src.depth());
  return false;
}

template<class _Tp>
static bool _average_pyramid_filter_up(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask,
    const cv::Mat & fallback_src, const cv::Mat1f & fallback_mask)
{
  INSTRUMENT_REGION("");

  constexpr cv::BorderTypes border_mode = cv::BORDER_REPLICATE;
  const cv::Size ksize(3, 3);

  parallel_invoke(
      [&] { cv::boxFilter(src, dst, src.depth(), ksize, cv::Point(-1, -1), false, border_mode); },
      [&] { cv::boxFilter(srcmask, dstmask, dstmask.depth(), ksize, cv::Point(-1, -1), false, border_mode); }
  );

  const cv::Size dstSize = dst.size();
  const int cn = dst.channels();

  uint8_t * dst_base = dst.ptr();
  const size_t dst_stride = dst.step;
  uint8_t * dstmask_base = dstmask.ptr();
  const size_t dstmask_stride = dstmask.step;

  const uint8_t * fbksrc_base = fallback_src.ptr();
  const size_t fbksrc_stride = fallback_src.step;
  const uint8_t * fbkmask_base = fallback_mask.ptr();
  const size_t fbkmask_stride = fallback_mask.step;

  parallel_for(0, dstSize.height, [=](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {

      _Tp * __restrict dstp = (_Tp * ) (dst_base + y * dst_stride);
      float * __restrict mskp = (float * ) (dstmask_base + y * dstmask_stride);

      const _Tp * fbksrc = (const _Tp * )(fbksrc_base + y * fbksrc_stride);
      const float * fbkmsk = (const float * )(fbkmask_base + y * fbkmask_stride);

      for ( int x = 0; x < dstSize.width; ++x, ++mskp, ++fbkmsk, dstp += cn, fbksrc += cn ) {
        const float m = *mskp;
        const float fm = *fbkmsk;
        const float scale = m ? 1.0f / m : 0.f;
        *mskp = (m || fm) ?  1.0f : 0.f;
        for ( int c = 0; c < cn; ++c ) {
          dstp[c] = fm ? fbksrc[c] : m ? cv::saturate_cast<_Tp>(dstp[c] * scale) : _Tp(0);
        }
      }
    }
  });

  return true;
}
static bool average_pyramid_filter_up(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask,
    const cv::Mat & fallback_src, const cv::Mat1f & fallback_mask)
{
  CV_DISPATCH(src.depth(), _average_pyramid_filter_up, src, srcmask, dst, dstmask, fallback_src, fallback_mask);
  CF_ERROR("Invalid src.depth()=%d", src.depth());
  return false;
}


template<class _Tp>
static bool _is_mask_fully_filled(cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  const int rows = _mask.rows();
  const int cols = _mask.cols();
  const cv::Mat mask = _mask.getMat();

  alignas(std::hardware_destructive_interference_size)
      std::atomic_bool found_empty(false);

  const uint8_t * mask_base = mask.ptr();
  const size_t mask_stride = mask.step;

  parallel_for(0, rows, [=, &found_empty](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      if ( found_empty.load(std::memory_order_relaxed) ) {
        break;
      }
      const _Tp * mskp = (const _Tp* )(mask_base + y * mask_stride);
      for ( int x = 0; x < cols; ++x ) {
        if ( !(mskp[x] > 0) ) {
          found_empty.store(true, std::memory_order_relaxed);
          break;
        }
      }
    }
  });

  return !found_empty.load();
}

static bool is_mask_fully_filled(cv::InputArray _mask)
{
  CV_DISPATCH(_mask.depth(), _is_mask_fully_filled, _mask);
  return true;
}

static void average_pyramid_recurse(cv::Mat & image, cv::Mat1f & mask, int max_levels)
{
  if ( std::min(image.cols, image.rows) > 1 && max_levels > 0 ) {
    INSTRUMENT_REGION("");

    cv::Mat filtered_image;
    cv::Mat1f filtered_mask;

    average_pyramid_filter_down(image, mask, filtered_image, filtered_mask);

    downstrike(filtered_image, filtered_image);
    downstrike(filtered_mask, filtered_mask);

    if ( !is_mask_fully_filled(filtered_mask) ) {
      average_pyramid_recurse(filtered_image, filtered_mask, max_levels - 1);
    }

    upject(filtered_image, filtered_image, image.size(), &filtered_mask);

    average_pyramid_filter_up(filtered_image, filtered_mask,
        filtered_image, filtered_mask,
        image, mask);

    image = std::move(filtered_image);
    mask = std::move(filtered_mask);
  }
}

template<class _Tp>
static bool _average_pyramid_init(cv::InputArray _src, cv::InputArray _srcmask,
    cv::OutputArray _dst, cv::OutputArray _dstmask)
{
  INSTRUMENT_REGION("");

  if ( _src.size() != _srcmask.size() ) {
    CF_ERROR("Input image and mask sizes differs");
    return false;
  }

  if ( _srcmask.type() != CV_8UC1 ) {
    CF_ERROR("Input mask must be of CV_8UC1 type");
    return false;
  }

  const cv::Size size = _src.size();
  const int cn = _src.channels();

  const cv::Mat src = _src.getMat();
  const cv::Mat mask = _srcmask.getMat();

  cv::Mat dst = createOutOfPlace(_src, _dst, size, CV_MAKETYPE(CV_32F, cn));
  cv::Mat dstmask = createOutOfPlace(_srcmask, _dstmask, size, CV_MAKETYPE(CV_32F, 1));

  const uint8_t * src_base = src.ptr();
  const size_t src_stride = src.step;

  const uint8_t * mask_base = mask.ptr();
  const size_t mask_stride = mask.step;

  uint8_t * dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  uint8_t * dstmask_base = dstmask.ptr();
  const size_t dstmask_stride = dstmask.step;

  parallel_for(0, size.height, [=](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {

      const _Tp * __restrict srcp = (const _Tp * ) (src_base + y * src_stride);
      const uint8_t * __restrict mskp = (const uint8_t * )(mask_base + y * mask_stride);

      float * __restrict dstp = (float * ) (dst_base + y * dst_stride);
      float * __restrict dstmskp = (float * ) (dstmask_base + y * dstmask_stride);

      for ( int x = 0; x < size.width; ++x, ++mskp, ++dstmskp ) {
        const uint8_t m = *mskp;
        *dstmskp = m ? 1.f : 0.f;
        for ( int c = 0; c < cn; ++c, ++srcp, ++dstp ) {
          *dstp = m ? *srcp : 0.f;
        }
      }
    }
  });

  assignOutOfPlace(_dst, dst);
  assignOutOfPlace(_dstmask, dstmask);
  return true;
}

static bool average_pyramid_init(cv::InputArray _src, cv::InputArray _srcmask,
    cv::OutputArray _dst, cv::OutputArray _dstmask)
{
  CV_DISPATCH(_src.depth(), _average_pyramid_init, _src, _srcmask, _dst, _dstmask);
  return false;
}

void average_pyramid_inpaint(cv::InputArray _src, cv::InputArray _mask,
    cv::OutputArray dst, cv::OutputArray _dstmask, int max_levels)
{
  INSTRUMENT_REGION("");

  if ( _mask.empty() || is_mask_fully_filled(_mask) ) {
    _src.copyTo(dst);
    if (_dstmask.needed()) {
      _mask.copyTo(_dstmask);
    }
    return;
  }

  cv::Mat src;
  cv::Mat1f msk;
  average_pyramid_init(_src, _mask, src, msk);
  average_pyramid_recurse(src, msk, max_levels);

  const int ddepth = dst.fixedType() ? dst.type() : _src.type();
  if (ddepth == src.depth() ) {
    dst.assign(src);
  }
  else {
    src.convertTo(dst, ddepth);
  }

  if (_dstmask.needed()) {
    msk.convertTo(_dstmask, CV_8U, 255);
  }
}

