/*
 * area_remap.cc
 *
 *  Created on: Nov 26, 2026
 *      Author: amyznikov
 */
#include "area_remap.h"
#include <core/proc/upscale_remap.h>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>


#ifndef ALWAYS_INLINE
#if defined(_MSC_VER)
#  define ALWAYS_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#  define ALWAYS_INLINE inline __attribute__((always_inline))
#else
#  define ALWAYS_INLINE inline
#endif
#endif // ALWAYS_INLINE

template<class _Tp1, int cn>
static bool __areaRemap(cv::InputArray srcImage, const cv::Mat2f & rmap, cv::OutputArray dstImage)
{
  INSTRUMENT_REGION("");

  const cv::Size & dstSize = rmap.size();

  const cv::Mat src = srcImage.getMat();
  cv::Mat dst = createOutOfPlace(srcImage, dstImage, dstSize, CV_MAKETYPE(cv::DataType<_Tp1>::depth, cn));

  const int src_rows = src.rows;
  const int src_cols = src.cols;
  const int dst_rows = dst.rows;
  const int dst_cols = dst.cols;
  const float step_x = float(src_cols) / float(dst_cols);
  const float step_y = float(src_rows) / float(dst_rows);

  // Ensure 1 pixel of the output image covers no more than 2x2 pixels of the src
  if ( step_x > 1.0f || step_y > 1.0f ) {
    CF_ERROR("Downscaling is not supported in 2x2 areaRemap (step_x=%f, step_y=%f)", step_x, step_y);
    return false;
  }

  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  const uint8_t * const rmap_base = rmap.ptr();
  const size_t rmap_stride = rmap.step;

  parallel_for(0, dst_rows, [=](const auto & range) {

    constexpr float drop_size = 1.f;
    constexpr float half_drop = 0.5f * drop_size;
    const float half_acc_x = 0.5f * step_x;
    const float half_acc_y = 0.5f * step_y;

    for ( int y = rbegin(range); y < rend(range); ++y ) {

      _Tp1 * __restrict dstp = (_Tp1 *)(dst_base + y * dst_stride);

      const cv::Vec2f * rmp1 = (const cv::Vec2f *)(rmap_base + y * rmap_stride);

      for ( int x = 0; x < dst_cols; ++x, dstp += cn ) {
        const cv::Vec2f p = rmp1[x];
        const float px = p(0);
        const float py = p(1);

        // Boundary validity flag
        const bool in_bounds = (px >= -0.5f) & (px <= src_cols - 0.5f) & (py >= -0.5f) & (py <= src_rows - 0.5f);

        // 2x2 neighborhood: sx0 <= px < sx1, sy0 <= py < sy1
        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
        const int sx1 = sx0 + 1, sy1 = sy0 + 1;

        // Destination pixel footprint boundaries in src coordinates
        const float ax0 = px - half_acc_x;
        const float ax1 = px + half_acc_x;
        const float ay0 = py - half_acc_y;
        const float ay1 = py + half_acc_y;

        // Exact 1D interval intersections for 2x2 neighborhood
        const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - std::max(ax0, sx0 - half_drop));
        const float ov_x1 = std::max(0.0f, std::min(ax1, sx1 + half_drop) - std::max(ax0, sx1 - half_drop));
        const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - std::max(ay0, sy0 - half_drop));
        const float ov_y1 = std::max(0.0f, std::min(ay1, sy1 + half_drop) - std::max(ay0, sy1 - half_drop));

        // Zero weights for outsides of the interval [-0.5, size - 0.5]
        const float valid_mask = in_bounds ? 1.0f : 0.0f;
        const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
        const float wx1 = (sx1 < src_cols) ? ov_x1 : 0.0f;
        const float wy0 = (sy0 >= 0) ? (ov_y0 * valid_mask) : 0.0f;
        const float wy1 = (sy1 < src_rows) ? (ov_y1 * valid_mask) : 0.0f;
        const float w00 = wy0 * wx0;
        const float w01 = wy0 * wx1;
        const float w10 = wy1 * wx0;
        const float w11 = wy1 * wx1;
        const float wsum = w00 + w01 + w10 + w11;
        const float winv = wsum > 0 ? 1.0f / wsum : 0.f;
        const float ww00 = w00 * winv;
        const float ww01 = w01 * winv;
        const float ww10 = w10 * winv;
        const float ww11 = w11 * winv;

        // Safe read indices
        const int ix0 = std::clamp(sx0, 0, src_cols - 1) * cn;
        const int iy0 = std::clamp(sy0, 0, src_rows - 1);
        const int ix1 = std::clamp(sx1, 0, src_cols - 1) * cn;
        const int iy1 = std::clamp(sy1, 0, src_rows - 1);

        const _Tp1 * const src0 = (const _Tp1 *)(src_base + iy0 * src_stride);
        const _Tp1 * const src1 = (const _Tp1 *)(src_base + iy1 * src_stride);
        const _Tp1 * const p00 = src0 + ix0;
        const _Tp1 * const p01 = src0 + ix1;
        const _Tp1 * const p10 = src1 + ix0;
        const _Tp1 * const p11 = src1 + ix1;

        for ( int c = 0; c < cn; ++c ) {
          const float val = (p00[c] * ww00 + p01[c] * ww01 + p10[c] * ww10 + p11[c] * ww11);
          dstp[c] = cv::saturate_cast<_Tp1>(val);
        }
      }
    }
  });

  assignOutOfPlace(dstImage, dst);

  return true;
}

template<class _Tp1,class _Tp2, int cn1, int cn2>
static bool __areaRemap2(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask)
{
  INSTRUMENT_REGION("");

  const cv::Size & dstSize = rmap.size();

  const cv::Mat src = srcImage.getMat();
  const cv::Mat srcm = srcMask.getMat();

  cv::Mat dst = createOutOfPlace(srcImage, dstImage, dstSize, CV_MAKETYPE(cv::DataType<_Tp1>::depth, cn1));
  cv::Mat dstm = createOutOfPlace(srcMask, dstMask, dstSize, CV_MAKETYPE(cv::DataType<_Tp2>::depth, cn2));

  const int src_rows = src.rows;
  const int src_cols = src.cols;
  const int dst_rows = dst.rows;
  const int dst_cols = dst.cols;
  const float step_x = float(src_cols) / float(dst_cols);
  const float step_y = float(src_rows) / float(dst_rows);

  // Ensure 1 pixel of the output image covers no more than 2x2 pixels of the src
  if ( step_x > 1.0f || step_y > 1.0f ) {
    CF_ERROR("Downscaling is not supported in 2x2 areaRemap (step_x=%f, step_y=%f)", step_x, step_y);
    return false;
  }

  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  const uint8_t * const srcm_base = srcm.empty() ? nullptr : srcm.ptr();
  const size_t srcm_stride = srcm.empty() ? 0 : srcm.step;

  uint8_t * const dstm_base = dstm.ptr();
  const size_t dstm_stride = dstm.step;

  const uint8_t * const rmap_base = rmap.ptr();
  const size_t rmap_stride = rmap.step;

  parallel_for(0, dst_rows, [=](const auto & range) {

    constexpr float drop_size = 1.f;
    constexpr float half_drop = 0.5f * drop_size;
    const float half_acc_x = 0.5f * step_x;
    const float half_acc_y = 0.5f * step_y;

    for ( int y = rbegin(range); y < rend(range); ++y ) {

      _Tp1 * __restrict dstp = (_Tp1 *)(dst_base + y * dst_stride);
      _Tp2 * __restrict dstmp = (_Tp2 *)(dstm_base + y * dstm_stride);

      const cv::Vec2f * rmp1 = (const cv::Vec2f *)(rmap_base + y * rmap_stride);

      for ( int x = 0; x < dst_cols; ++x, dstp += cn1, dstmp += cn2) {
        const cv::Vec2f p = rmp1[x];
        const float px = p(0);
        const float py = p(1);

        // Boundary validity flag
        const bool in_bounds = (px >= -0.5f) & (px <= src_cols - 0.5f) & (py >= -0.5f) & (py <= src_rows - 0.5f);

        // 2x2 neighborhood: sx0 <= px < sx1, sy0 <= py < sy1
        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
        const int sx1 = sx0 + 1, sy1 = sy0 + 1;

        // Destination pixel footprint boundaries in src coordinates
        const float ax0 = px - half_acc_x;
        const float ax1 = px + half_acc_x;
        const float ay0 = py - half_acc_y;
        const float ay1 = py + half_acc_y;

        // Exact 1D interval intersections for 2x2 neighborhood
        const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - std::max(ax0, sx0 - half_drop));
        const float ov_x1 = std::max(0.0f, std::min(ax1, sx1 + half_drop) - std::max(ax0, sx1 - half_drop));
        const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - std::max(ay0, sy0 - half_drop));
        const float ov_y1 = std::max(0.0f, std::min(ay1, sy1 + half_drop) - std::max(ay0, sy1 - half_drop));

        // Zero weights for outsides of the interval [-0.5, size - 0.5]
        const float valid_mask = in_bounds ? 1.0f : 0.0f;
        const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
        const float wx1 = (sx1 < src_cols) ? ov_x1 : 0.0f;
        const float wy0 = (sy0 >= 0) ? (ov_y0 * valid_mask) : 0.0f;
        const float wy1 = (sy1 < src_rows) ? (ov_y1 * valid_mask) : 0.0f;
        const float w00 = wy0 * wx0;
        const float w01 = wy0 * wx1;
        const float w10 = wy1 * wx0;
        const float w11 = wy1 * wx1;
        const float wsum = w00 + w01 + w10 + w11;
        const float winv = wsum > 0 ? 1.0f / wsum : 0.f;
        const float ww00 = w00 * winv;
        const float ww01 = w01 * winv;
        const float ww10 = w10 * winv;
        const float ww11 = w11 * winv;

        // Safe read indices
        const int ix0 = std::clamp(sx0, 0, src_cols - 1);
        const int iy0 = std::clamp(sy0, 0, src_rows - 1);
        const int ix1 = std::clamp(sx1, 0, src_cols - 1);
        const int iy1 = std::clamp(sy1, 0, src_rows - 1);

        const _Tp1 * const src0 = (const _Tp1 *)(src_base + iy0 * src_stride);
        const _Tp1 * const src1 = (const _Tp1 *)(src_base + iy1 * src_stride);
        const _Tp1 * const s00 = src0 + ix0 * cn1;
        const _Tp1 * const s01 = src0 + ix1 * cn1;
        const _Tp1 * const s10 = src1 + ix0 * cn1;
        const _Tp1 * const s11 = src1 + ix1 * cn1;
        for ( int c = 0; c < cn1; ++c ) {
          const float val = (s00[c] * ww00 + s01[c] * ww01 + s10[c] * ww10 + s11[c] * ww11);
          dstp[c] = cv::saturate_cast<_Tp1>(val);
        }

        if ( !srcm_base ) {
          constexpr _Tp2 M = std::is_same_v<_Tp2, uint8_t> ? _Tp2(255) : _Tp2(1);
          const bool valid_pixel = in_bounds & (wsum > 0.0f);
          for ( int c = 0; c < cn2; ++c ) {
            dstmp[c] = valid_pixel ? M : 0;
          }
        }
        else {
          const _Tp2 * const msk0 = (const _Tp2 *)(srcm_base + iy0 * srcm_stride);
          const _Tp2 * const msk1 = (const _Tp2 *)(srcm_base + iy1 * srcm_stride);
          const _Tp2 * const m00 = msk0 + ix0 * cn2;
          const _Tp2 * const m01 = msk0 + ix1 * cn2;
          const _Tp2 * const m10 = msk1 + ix0 * cn2;
          const _Tp2 * const m11 = msk1 + ix1 * cn2;
          for ( int c = 0; c < cn2; ++c ) {
            const float val = (m00[c] * ww00 + m01[c] * ww01 + m10[c] * ww10 + m11[c] * ww11);
            dstmp[c] = cv::saturate_cast<_Tp2>(val);
          }
        }
      }
    }
  });


  assignOutOfPlace(dstImage, dst);
  assignOutOfPlace(dstMask, dstm);

  return true;
}

template<class _Tp1>
static bool _areaRemap(cv::InputArray srcImage, const cv::Mat2f & rmap, cv::OutputArray dstImage)
{
  switch(srcImage.channels())
  {
    case 1: return __areaRemap<_Tp1, 1>(srcImage, rmap, dstImage);
    case 2: return __areaRemap<_Tp1, 2>(srcImage, rmap, dstImage);
    case 3: return __areaRemap<_Tp1, 3>(srcImage, rmap, dstImage);
    case 4: return __areaRemap<_Tp1, 4>(srcImage, rmap, dstImage);
  }

  return false;
}

bool areaRemap(cv::InputArray srcImage, const cv::Mat2f & rmap, cv::OutputArray dstImage)
{
  if ( srcImage.empty() ) {
    CF_ERROR("Empty input image");
    return false;
  }

  const cv::Size srcSize = srcImage.size();
  const cv::Size dstSize = rmap.size();

  if( dstImage.fixedType() && dstImage.type() != srcImage.type() ) {
    CF_ERROR("Bad output image fixed type. Must be same as source type");
    return false;
  }

  CV_DISPATCH(srcImage.depth(), _areaRemap, srcImage, rmap, dstImage);

  return false;
}


template<class _Tp1, class _Tp2>
static bool _areaRemap2(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask)
{
  const int cn1 = srcImage.channels();
  const int cn2 = srcMask.empty() ? 1 : srcMask.channels();

  switch(cn1)
  {
    case 1: switch(cn2) {
      case 1: return __areaRemap2<_Tp1, _Tp2, 1, 1>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 2: return __areaRemap2<_Tp1, _Tp2, 1, 2>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 3: return __areaRemap2<_Tp1, _Tp2, 1, 3>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 4: return __areaRemap2<_Tp1, _Tp2, 1, 4>(srcImage, srcMask, rmap, dstImage, dstMask);
    }
    break;

    case 2: switch(cn2) {
      case 1: return __areaRemap2<_Tp1, _Tp2, 2, 1>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 2: return __areaRemap2<_Tp1, _Tp2, 2, 2>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 3: return __areaRemap2<_Tp1, _Tp2, 2, 3>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 4: return __areaRemap2<_Tp1, _Tp2, 2, 4>(srcImage, srcMask, rmap, dstImage, dstMask);
    }
    break;

    case 3: switch(cn2) {
      case 1: return __areaRemap2<_Tp1, _Tp2, 3, 1>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 2: return __areaRemap2<_Tp1, _Tp2, 3, 2>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 3: return __areaRemap2<_Tp1, _Tp2, 3, 3>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 4: return __areaRemap2<_Tp1, _Tp2, 3, 4>(srcImage, srcMask, rmap, dstImage, dstMask);
    }
    break;

    case 4: switch(cn2) {
      case 1: return __areaRemap2<_Tp1, _Tp2, 4, 1>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 2: return __areaRemap2<_Tp1, _Tp2, 4, 2>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 3: return __areaRemap2<_Tp1, _Tp2, 4, 3>(srcImage, srcMask, rmap, dstImage, dstMask);
      case 4: return __areaRemap2<_Tp1, _Tp2, 4, 4>(srcImage, srcMask, rmap, dstImage, dstMask);
    }
    break;
  }

  return false;
}

bool areaRemap(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask, int mdepth)
{
  if ( !dstMask.needed() ) {
    return areaRemap(srcImage, rmap, dstImage);
  }

  if ( srcImage.empty() ) {
    CF_ERROR("Empty input image");
    return false;
  }

  const cv::Size srcSize = srcImage.size();
  const cv::Size dstSize = rmap.size();

  if( dstImage.fixedType() && dstImage.type() != srcImage.type() ) {
    CF_ERROR("Bad output image fixed type. Must be same as source type");
    return false;
  }

  if ( mdepth < 0 ) {
    if ( !srcMask.empty() ) {
      mdepth = srcMask.depth();
    }
    else if ( dstMask.fixedType() ) {
      mdepth = dstMask.depth();
    }
    else {
      mdepth = CV_8U;
    }
  }

  if( dstMask.fixedType() && dstMask.depth() != mdepth ) {
    CF_ERROR("Bad output mask fixed type. Must be same as source mask type");
    return false;
  }


  CV_DISPATCH2(srcImage.depth(), mdepth, _areaRemap2,
      srcImage, srcMask, rmap, dstImage, dstMask);

  return false;
}
