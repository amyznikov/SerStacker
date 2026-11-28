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
static bool __areaRemap(cv::InputArray srcImage, const cv::Mat2f & rmap, cv::OutputArray dstImage, float drop_size)
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

    //constexpr float drop_size = 1.f;
    const float half_drop = 0.5f * drop_size;
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

template<class _Tp1>
static bool _areaRemap(cv::InputArray srcImage, const cv::Mat2f & rmap, cv::OutputArray dstImage, float drop_size)
{
  switch(srcImage.channels())
  {
    case 1: return __areaRemap<_Tp1, 1>(srcImage, rmap, dstImage, drop_size);
    case 2: return __areaRemap<_Tp1, 2>(srcImage, rmap, dstImage, drop_size);
    case 3: return __areaRemap<_Tp1, 3>(srcImage, rmap, dstImage, drop_size);
    case 4: return __areaRemap<_Tp1, 4>(srcImage, rmap, dstImage, drop_size);
  }

  return false;
}

bool areaRemap(cv::InputArray srcImage, const cv::Mat2f & rmap, cv::OutputArray dstImage, float drop_size)
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

  CV_DISPATCH(srcImage.depth(), _areaRemap, srcImage, rmap, dstImage, drop_size);

  return false;
}

namespace {

enum class SrcMaskMode {
  None,
  U8C1,
  F32C1
};

enum class DstMaskMode {
  None,
  U8C1,
  F32C1,
  F32CN
};

} // namespace

template<class _Tp1, int cn>
static bool __areaRemap2_scalar_mask(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask,
    SrcMaskMode src_mode, DstMaskMode dst_mode, int out_mtype, float drop_size)
{
  INSTRUMENT_REGION("");

  const cv::Size dstSize = rmap.size();
  const cv::Mat src = srcImage.getMat();
  const cv::Mat srcm = (src_mode != SrcMaskMode::None) ? srcMask.getMat() : cv::Mat();

  cv::Mat dst = createOutOfPlace(srcImage, dstImage, dstSize, CV_MAKETYPE(cv::DataType<_Tp1>::depth, cn));

  cv::Mat dstm;
  if ( dst_mode != DstMaskMode::None ) {
    dstm = createOutOfPlace(srcMask, dstMask, dstSize, out_mtype);
  }

  const int src_rows = src.rows;
  const int src_cols = src.cols;
  const int dst_rows = dst.rows;
  const int dst_cols = dst.cols;
  const float step_x = float(src_cols) / float(dst_cols);
  const float step_y = float(src_rows) / float(dst_rows);

  if ( step_x > 1.0f || step_y > 1.0f ) {
    CF_ERROR("Downscaling is not supported in 2x2 areaRemap (step_x=%f, step_y=%f)", step_x, step_y);
    return false;
  }

  const float max_drop = 2.0f - std::max(step_x, step_y);
  if ( drop_size <= 0.0f || drop_size > max_drop ) {
    CF_ERROR("Invalid drop_size=%f (must be in (0, %f])", drop_size, max_drop);
    return false;
  }

  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  const uint8_t * const srcm_base = srcm.empty() ? nullptr : srcm.ptr();
  const size_t srcm_stride = srcm.empty() ? 0 : srcm.step;

  uint8_t * const dstm_base = dstm.empty() ? nullptr : dstm.ptr();
  const size_t dstm_stride = dstm.empty() ? 0 : dstm.step;

  const uint8_t * const rmap_base = rmap.ptr();
  const size_t rmap_stride = rmap.step;

  parallel_for(0, dst_rows, [=](const auto & range) {

    const float half_drop = 0.5f * drop_size;
    const float half_acc_x = 0.5f * step_x;
    const float half_acc_y = 0.5f * step_y;
    const float inv_pixel_area = 1.0f / (step_x * step_y);
    constexpr float u8_scale = 1.0f / 255.0f;

    for ( int y = rbegin(range); y < rend(range); ++y ) {

      _Tp1 * __restrict dstp = (_Tp1 *)(dst_base + y * dst_stride);
      uint8_t * __restrict dstm_row = dstm_base ? (dstm_base + y * dstm_stride) : nullptr;
      const cv::Vec2f * rmp1 = (const cv::Vec2f *)(rmap_base + y * rmap_stride);

      for ( int x = 0; x < dst_cols; ++x, dstp += cn ) {
        const cv::Vec2f p = rmp1[x];
        const float px = p(0);
        const float py = p(1);

        const bool in_bounds = (px >= -0.5f) & (px <= src_cols - 0.5f) & (py >= -0.5f) & (py <= src_rows - 0.5f);

        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
        const int sx1 = sx0 + 1, sy1 = sy0 + 1;

        const float ax0 = px - half_acc_x;
        const float ax1 = px + half_acc_x;
        const float ay0 = py - half_acc_y;
        const float ay1 = py + half_acc_y;

        const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - std::max(ax0, sx0 - half_drop));
        const float ov_x1 = std::max(0.0f, std::min(ax1, sx1 + half_drop) - std::max(ax0, sx1 - half_drop));
        const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - std::max(ay0, sy0 - half_drop));
        const float ov_y1 = std::max(0.0f, std::min(ay1, sy1 + half_drop) - std::max(ay0, sy1 - half_drop));

        const float valid_mask = in_bounds ? 1.0f : 0.0f;
        const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
        const float wx1 = (sx1 < src_cols) ? ov_x1 : 0.0f;
        const float wy0 = (sy0 >= 0) ? (ov_y0 * valid_mask) : 0.0f;
        const float wy1 = (sy1 < src_rows) ? (ov_y1 * valid_mask) : 0.0f;

        float w00 = wy0 * wx0;
        float w01 = wy0 * wx1;
        float w10 = wy1 * wx0;
        float w11 = wy1 * wx1;

        const int ix0 = std::clamp(sx0, 0, src_cols - 1);
        const int iy0 = std::clamp(sy0, 0, src_rows - 1);
        const int ix1 = std::clamp(sx1, 0, src_cols - 1);
        const int iy1 = std::clamp(sy1, 0, src_rows - 1);

        if ( src_mode == SrcMaskMode::U8C1 ) {
          const uint8_t * const m0 = srcm_base + iy0 * srcm_stride;
          const uint8_t * const m1 = srcm_base + iy1 * srcm_stride;
          w00 *= m0[ix0] * u8_scale;
          w01 *= m0[ix1] * u8_scale;
          w10 *= m1[ix0] * u8_scale;
          w11 *= m1[ix1] * u8_scale;
        }
        else if ( src_mode == SrcMaskMode::F32C1 ) {
          const float * const m0 = (const float *)(srcm_base + iy0 * srcm_stride);
          const float * const m1 = (const float *)(srcm_base + iy1 * srcm_stride);
          w00 *= m0[ix0];
          w01 *= m0[ix1];
          w10 *= m1[ix0];
          w11 *= m1[ix1];
        }

        const float wsum = w00 + w01 + w10 + w11;
        const float winv = wsum > 0.0f ? 1.0f / wsum : 0.0f;
        const float ww00 = w00 * winv;
        const float ww01 = w01 * winv;
        const float ww10 = w10 * winv;
        const float ww11 = w11 * winv;

        const _Tp1 * const src0 = (const _Tp1 *)(src_base + iy0 * src_stride);
        const _Tp1 * const src1 = (const _Tp1 *)(src_base + iy1 * src_stride);
        const _Tp1 * const s00 = src0 + ix0 * cn;
        const _Tp1 * const s01 = src0 + ix1 * cn;
        const _Tp1 * const s10 = src1 + ix0 * cn;
        const _Tp1 * const s11 = src1 + ix1 * cn;

        for ( int c = 0; c < cn; ++c ) {
          const float val = s00[c] * ww00 + s01[c] * ww01 + s10[c] * ww10 + s11[c] * ww11;
          dstp[c] = cv::saturate_cast<_Tp1>(val);
        }

        if ( dst_mode == DstMaskMode::U8C1 ) {
          dstm_row[x] = (in_bounds & (wsum > 0.0f)) ? 255 : 0;
        }
        else if ( dst_mode == DstMaskMode::F32C1 ) {
          ((float *)dstm_row)[x] = wsum * inv_pixel_area;
        }
        else if ( dst_mode == DstMaskMode::F32CN ) {
          const float cov = wsum * inv_pixel_area;
          float * const out_m = ((float *)dstm_row) + x * cn;
          for ( int c = 0; c < cn; ++c ) {
            out_m[c] = cov;
          }
        }
      }
    }
  });

  assignOutOfPlace(dstImage, dst);
  if ( dst_mode != DstMaskMode::None ) {
    assignOutOfPlace(dstMask, dstm);
  }

  return true;
}

template<class _Tp1, int cn>
static bool __areaRemap2_planar_mask(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask,
    DstMaskMode dst_mode, int out_mtype, float drop_size)
{
  INSTRUMENT_REGION("");

  const cv::Size & dstSize = rmap.size();
  const cv::Mat src = srcImage.getMat();
  const cv::Mat srcm = srcMask.getMat();

  cv::Mat dst = createOutOfPlace(srcImage, dstImage, dstSize, CV_MAKETYPE(cv::DataType<_Tp1>::depth, cn));
  cv::Mat dstm;
  if ( dst_mode != DstMaskMode::None ) {
    dstm = createOutOfPlace(srcMask, dstMask, dstSize, out_mtype);
  }

  const int src_rows = src.rows;
  const int src_cols = src.cols;
  const int dst_rows = dst.rows;
  const int dst_cols = dst.cols;
  const float step_x = float(src_cols) / float(dst_cols);
  const float step_y = float(src_rows) / float(dst_rows);

  if ( step_x > 1.0f || step_y > 1.0f ) {
    CF_ERROR("Downscaling is not supported in 2x2 areaRemap (step_x=%f, step_y=%f)", step_x, step_y);
    return false;
  }

  const float max_drop = 2.0f - std::max(step_x, step_y);
  if ( drop_size <= 0.0f || drop_size > max_drop ) {
    CF_ERROR("Invalid drop_size=%f (must be in (0, %f])", drop_size, max_drop);
    return false;
  }

  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  const uint8_t * const srcm_base = srcm.ptr();
  const size_t srcm_stride = srcm.step;

  uint8_t * const dstm_base = dstm.empty() ? nullptr : dstm.ptr();
  const size_t dstm_stride = dstm.empty() ? 0 : dstm.step;

  const uint8_t * const rmap_base = rmap.ptr();
  const size_t rmap_stride = rmap.step;

  parallel_for(0, dst_rows, [=](const auto & range) {

    const float half_drop = 0.5f * drop_size;
    const float half_acc_x = 0.5f * step_x;
    const float half_acc_y = 0.5f * step_y;
    const float inv_pixel_area = 1.0f / (step_x * step_y);

    for ( int y = rbegin(range); y < rend(range); ++y ) {

      _Tp1 * __restrict dstp = (_Tp1 *)(dst_base + y * dst_stride);
      uint8_t * __restrict dstm_row = dstm_base ? (dstm_base + y * dstm_stride) : nullptr;
      const cv::Vec2f * rmp1 = (const cv::Vec2f *)(rmap_base + y * rmap_stride);

      for ( int x = 0; x < dst_cols; ++x, dstp += cn ) {
        const cv::Vec2f p = rmp1[x];
        const float px = p(0);
        const float py = p(1);

        const bool in_bounds = (px >= -0.5f) & (px <= src_cols - 0.5f) & (py >= -0.5f) & (py <= src_rows - 0.5f);

        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
        const int sx1 = sx0 + 1, sy1 = sy0 + 1;

        const float ax0 = px - half_acc_x;
        const float ax1 = px + half_acc_x;
        const float ay0 = py - half_acc_y;
        const float ay1 = py + half_acc_y;

        const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - std::max(ax0, sx0 - half_drop));
        const float ov_x1 = std::max(0.0f, std::min(ax1, sx1 + half_drop) - std::max(ax0, sx1 - half_drop));
        const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - std::max(ay0, sy0 - half_drop));
        const float ov_y1 = std::max(0.0f, std::min(ay1, sy1 + half_drop) - std::max(ay0, sy1 - half_drop));

        const float valid_mask = in_bounds ? 1.0f : 0.0f;
        const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
        const float wx1 = (sx1 < src_cols) ? ov_x1 : 0.0f;
        const float wy0 = (sy0 >= 0) ? (ov_y0 * valid_mask) : 0.0f;
        const float wy1 = (sy1 < src_rows) ? (ov_y1 * valid_mask) : 0.0f;

        const float g00 = wy0 * wx0;
        const float g01 = wy0 * wx1;
        const float g10 = wy1 * wx0;
        const float g11 = wy1 * wx1;

        const int ix0 = std::clamp(sx0, 0, src_cols - 1) * cn;
        const int iy0 = std::clamp(sy0, 0, src_rows - 1);
        const int ix1 = std::clamp(sx1, 0, src_cols - 1) * cn;
        const int iy1 = std::clamp(sy1, 0, src_rows - 1);

        const _Tp1 * const src0 = (const _Tp1 *)(src_base + iy0 * src_stride);
        const _Tp1 * const src1 = (const _Tp1 *)(src_base + iy1 * src_stride);
        const _Tp1 * const s00 = src0 + ix0;
        const _Tp1 * const s01 = src0 + ix1;
        const _Tp1 * const s10 = src1 + ix0;
        const _Tp1 * const s11 = src1 + ix1;

        const float * const msk0 = (const float *)(srcm_base + iy0 * srcm_stride);
        const float * const msk1 = (const float *)(srcm_base + iy1 * srcm_stride);
        const float * const m00 = msk0 + ix0;
        const float * const m01 = msk0 + ix1;
        const float * const m10 = msk1 + ix0;
        const float * const m11 = msk1 + ix1;

        float wsums[cn];

        for ( int c = 0; c < cn; ++c ) {
          const float w00 = g00 * m00[c];
          const float w01 = g01 * m01[c];
          const float w10 = g10 * m10[c];
          const float w11 = g11 * m11[c];

          const float wsum = w00 + w01 + w10 + w11;
          const float winv = wsum > 0.0f ? 1.0f / wsum : 0.0f;

          const float val = (s00[c] * w00 + s01[c] * w01 + s10[c] * w10 + s11[c] * w11) * winv;
          dstp[c] = cv::saturate_cast<_Tp1>(val);
          wsums[c] = wsum;
        }

        if ( dst_mode == DstMaskMode::F32CN ) {
          float * const out_m = ((float *)dstm_row) + x * cn;
          for ( int c = 0; c < cn; ++c ) {
            out_m[c] = wsums[c] * inv_pixel_area;
          }
        }
        else if ( dst_mode == DstMaskMode::U8C1 ) {
          float wsum_any = 0.0f;
          for ( int c = 0; c < cn; ++c ) {
            wsum_any += wsums[c];
          }
          dstm_row[x] = (in_bounds & (wsum_any > 0.0f)) ? 255 : 0;
        }
      }
    }
  });

  assignOutOfPlace(dstImage, dst);
  if ( dst_mode != DstMaskMode::None ) {
    assignOutOfPlace(dstMask, dstm);
  }

  return true;
}

template<class _Tp1>
static bool _areaRemap2(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask,
    SrcMaskMode src_mode, DstMaskMode dst_mode, int out_mtype,
    float drop_size)
{
  const int cn = srcImage.channels();

  if ( !srcMask.empty() && srcMask.channels() > 1 ) {
    switch ( cn ) {
      case 2: return __areaRemap2_planar_mask<_Tp1, 2>(srcImage, srcMask, rmap, dstImage, dstMask, dst_mode, out_mtype, drop_size);
      case 3: return __areaRemap2_planar_mask<_Tp1, 3>(srcImage, srcMask, rmap, dstImage, dstMask, dst_mode, out_mtype, drop_size);
      case 4: return __areaRemap2_planar_mask<_Tp1, 4>(srcImage, srcMask, rmap, dstImage, dstMask, dst_mode, out_mtype, drop_size);
    }
    return false;
  }

  switch ( cn ) {
    case 1: return __areaRemap2_scalar_mask<_Tp1, 1>(srcImage, srcMask, rmap, dstImage, dstMask, src_mode, dst_mode, out_mtype, drop_size);
    case 2: return __areaRemap2_scalar_mask<_Tp1, 2>(srcImage, srcMask, rmap, dstImage, dstMask, src_mode, dst_mode, out_mtype, drop_size);
    case 3: return __areaRemap2_scalar_mask<_Tp1, 3>(srcImage, srcMask, rmap, dstImage, dstMask, src_mode, dst_mode, out_mtype, drop_size);
    case 4: return __areaRemap2_scalar_mask<_Tp1, 4>(srcImage, srcMask, rmap, dstImage, dstMask, src_mode, dst_mode, out_mtype, drop_size);
  }
  return false;
}

bool areaRemap(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask, int mtype, float drop_size)
{
  if ( srcMask.empty() && !dstMask.needed() ) {
    return areaRemap(srcImage, rmap, dstImage, drop_size);
  }

  if ( srcImage.empty() ) {
    CF_ERROR("Empty input image");
    return false;
  }

  const int cn = srcImage.channels();

  if ( dstImage.fixedType() && dstImage.type() != srcImage.type() ) {
    CF_ERROR("Bad output image fixed type. Must be same as source type");
    return false;
  }

  SrcMaskMode src_mode = SrcMaskMode::None;
  if ( !srcMask.empty() ) {
    if ( srcMask.size() != srcImage.size() ) {
      CF_ERROR("srcMask size (%dx%d) does not match srcImage size (%dx%d)",
          srcMask.cols(), srcMask.rows(), srcImage.cols(), srcImage.rows());
      return false;
    }

    const int smtype = srcMask.type();
    if ( smtype == CV_8UC1 ) {
      src_mode = SrcMaskMode::U8C1;
    }
    else if ( smtype == CV_32FC1 ) {
      src_mode = SrcMaskMode::F32C1;
    }
    else if ( smtype == CV_32FC(cn) ) {
      src_mode = SrcMaskMode::F32C1; // handled by planar branch when channels > 1
    }
    else {
      CF_ERROR("Unsupported srcMask type: %d. Must be CV_8UC1, CV_32FC1 or CV_32FC(%d)", smtype, cn);
      return false;
    }
  }

  DstMaskMode dst_mode = DstMaskMode::None;
  int out_mtype = -1;

  if ( dstMask.needed() ) {
    if ( dstMask.fixedType() ) {
      if ( mtype >= 0 && mtype != dstMask.type() ) {
        CF_ERROR("Requested mtype (%d) conflicts with dstMask fixed type (%d)", mtype, dstMask.type());
        return false;
      }
      out_mtype = dstMask.type();
    }
    else if ( mtype >= 0 ) {
      out_mtype = mtype;
    }
    else if ( !srcMask.empty() ) {
      out_mtype = srcMask.type();
    }
    else {
      out_mtype = CV_32FC1;
    }

    if ( out_mtype == CV_8UC1 ) {
      dst_mode = DstMaskMode::U8C1;
    }
    else if ( out_mtype == CV_32FC1 ) {
      dst_mode = DstMaskMode::F32C1;
    }
    else if ( out_mtype == CV_32FC(cn) ) {
      dst_mode = DstMaskMode::F32CN;
    }
    else {
      CF_ERROR("Unsupported dstMask type: %d. Must be CV_8UC1, CV_32FC1 or CV_32FC(%d)", out_mtype, cn);
      return false;
    }

    if ( !srcMask.empty() && srcMask.channels() > 1 && out_mtype == CV_32FC1 ) {
      CF_ERROR("Implicit reduction from multi-channel srcMask (CV_32FC%d) to single-channel dstMask (CV_32FC1) is not allowed", cn);
      return false;
    }
  }

  CV_DISPATCH(srcImage.depth(), _areaRemap2, srcImage, srcMask, rmap,
      dstImage, dstMask, src_mode, dst_mode,
      out_mtype, drop_size);

  return false;
}
