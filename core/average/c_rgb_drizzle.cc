/*
 * c_rgb_drizzle.cc
 *
 *  Created on: Nov 28, 2026
 *      Author: amyznikov
 */
#include "c_frame_accumulation.h"
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/ssprintf.h>
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


namespace {
  enum class RmapMode {
    Empty,
    Direct, // rmap.size() == acc_size
    Scaled  // rmap.size() == cfa_size (on-fly interpolation)
  };
}

// weights for the Catmull-Rom bicubic spline (a = -0.5) for t in [0, 1]
ALWAYS_INLINE
static void catmull_rom_weights(float t, float w[4])
{
  const float t2 = t * t;
  const float t3 = t * t * t;
  w[0] = -0.5f * t + t2 - 0.5f * t3;
  w[1] = 1.0f - 2.5f * t2 + 1.5f * t3;
  w[2] = 0.5f * t + 2.0f * t2 - 1.5f * t3;
  w[3] = -0.5f * t2 + 0.5f * t3;
}

//template<class _Tp, class _Tpw, RmapMode _RmapMode>
//static bool _rgb_drizzle_core(cv::InputArray _src_image,
//    cv::Mat3f & acc, cv::Mat3f & cntr,
//    const cv::Mat2f & rmap,
//    cv::InputArray weights)
//{
//  const cv::Mat_<_Tp> src_image = _src_image.getMat();
//  const uint8_t * const src_base = src_image.ptr();
//  const size_t src_stride = src_image.step;
//
//  const uint8_t * const acc_base = acc.ptr();
//  const size_t acc_stride = acc.step;
//
//  const uint8_t * const cntr_base = cntr.ptr();
//  const size_t cntr_stride = cntr.step;
//
//  constexpr bool has_weights = !std::is_void_v<_Tpw>;
//  const cv::Mat wmap = has_weights ? weights.getMat() : cv::Mat();
//  const uint8_t * const wmap_base = has_weights ? wmap.ptr() : nullptr;
//  const size_t wmap_stride = has_weights ? wmap.step : 0;
//
//  const uint8_t * const rmap_base = (_RmapMode != RmapMode::Empty) ? rmap.ptr() : nullptr;
//  const size_t rmap_stride = (_RmapMode != RmapMode::Empty) ? rmap.step : 0;
//
//  const int src_rows = src_image.rows;
//  const int src_cols = src_image.cols;
//  const int acc_rows = acc.rows;
//  const int acc_cols = acc.cols;
//
//  const float step_x = float(src_cols) / float(acc_cols);
//  const float step_y = float(src_rows) / float(acc_rows);
//
//  // Ensure 1 pixel of the sensor array covers no more than 2x2 pixels of the CFA
//  if ( step_x > 1.0f || step_y > 1.0f ) {
//    CF_ERROR("Downscaling is not supported in 2x2 drizzle (step_x=%f, step_y=%f)", step_x, step_y);
//    return false;
//  }
//
//  parallel_for(0, acc_rows, [=](const auto & range) {
//
//    // Hardcoded pixfrac = 1 / scale = min(step_x, step_y)
//    const float half_drop = 0.5f * std::max(1e-2f, std::min(step_x, step_y));
//    const float half_acc_x = 0.5f * step_x;
//    const float half_acc_y = 0.5f * step_y;
//
//    const float offset_x = 0.5f * step_x - 0.5f;
//    const float offset_y = 0.5f * step_y - 0.5f;
//
//    for ( int y = rbegin(range); y < rend(range); ++y ) {
//      cv::Vec3f * __restrict accp = (cv::Vec3f *)(acc_base + y * acc_stride);
//      cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
//      const float rmap_y = y * step_y + offset_y;
//
//      const cv::Vec2f * rmp0 = nullptr;
//      const cv::Vec2f * rmp1 = nullptr;
//      const cv::Vec2f * rmp2 = nullptr;
//      const cv::Vec2f * rmp3 = nullptr;
//      float rwy[4] = {0.0f, 1.0f, 0.0f, 0.0f};
//
//      if constexpr ( _RmapMode == RmapMode::Direct ) {
//        // remap.size() == acc.size()
//        rmp1 = (const cv::Vec2f *)(rmap_base + y * rmap_stride);
//      }
//      else if constexpr ( _RmapMode == RmapMode::Scaled ) {
//        // remap.size() == cfa_image.size()
//        const float clamped_rmap_y = std::clamp(rmap_y, 0.0f, float(src_rows - 1));
//        const int ry1 = cvFloor(clamped_rmap_y);
//        const int ry0 = std::max(ry1 - 1, 0);
//        const int ry2 = std::min(ry1 + 1, src_rows - 1);
//        const int ry3 = std::min(ry1 + 2, src_rows - 1);
//
//        catmull_rom_weights(clamped_rmap_y - ry1, rwy);
//
//        rmp0 = (const cv::Vec2f *)(rmap_base + ry0 * rmap_stride);
//        rmp1 = (const cv::Vec2f *)(rmap_base + ry1 * rmap_stride);
//        rmp2 = (const cv::Vec2f *)(rmap_base + ry2 * rmap_stride);
//        rmp3 = (const cv::Vec2f *)(rmap_base + ry3 * rmap_stride);
//      }
//
//      for ( int x = 0; x < acc_cols; ++x ) {
//        const float rmap_x = x * step_x + offset_x;
//
//        float px, py;
//        if constexpr ( _RmapMode == RmapMode::Empty ) {
//          // no remap
//          px = rmap_x;
//          py = rmap_y;
//        }
//        else if constexpr ( _RmapMode == RmapMode::Direct ) {
//          // remap.size() == acc.size()
//          const cv::Vec2f p = rmp1[x];
//          px = p[0];
//          py = p[1];
//        }
//        else { // RmapMode::Scaled (Bicubic Catmull-Rom)
//          // remap.size() == cfa_image.size()
//          const float rx = std::clamp(rmap_x, 0.0f, float(src_cols - 1));
//          const int rx1 = cvFloor(rx);
//          const int rx0 = std::max(rx1 - 1, 0);
//          const int rx2 = std::min(rx1 + 1, src_cols - 1);
//          const int rx3 = std::min(rx1 + 2, src_cols - 1);
//
//          float rwx[4];
//          catmull_rom_weights(rx - rx1, rwx);
//
//          const cv::Vec2f row0 = rmp0[rx0] * rwx[0] + rmp0[rx1] * rwx[1] + rmp0[rx2] * rwx[2] + rmp0[rx3] * rwx[3];
//          const cv::Vec2f row1 = rmp1[rx0] * rwx[0] + rmp1[rx1] * rwx[1] + rmp1[rx2] * rwx[2] + rmp1[rx3] * rwx[3];
//          const cv::Vec2f row2 = rmp2[rx0] * rwx[0] + rmp2[rx1] * rwx[1] + rmp2[rx2] * rwx[2] + rmp2[rx3] * rwx[3];
//          const cv::Vec2f row3 = rmp3[rx0] * rwx[0] + rmp3[rx1] * rwx[1] + rmp3[rx2] * rwx[2] + rmp3[rx3] * rwx[3];
//          const cv::Vec2f p = row0 * rwy[0] + row1 * rwy[1] + row2 * rwy[2] + row3 * rwy[3];
//          px = p[0];
//          py = p[1];
//        }
//
//        // Account for an external half-pixel border [-0.5, cols-0.5]
//        if ( px < -0.5f || px > src_cols - 0.5f || py < -0.5f || py > src_rows - 0.5f ) {
//          continue;
//        }
//
//        float w;
//        if constexpr ( !has_weights ) {
//          w = 1.0f;
//        }
//        else {
//          const int wy_idx = std::clamp(cvRound(py), 0, src_rows - 1);
//          const int wx_idx = std::clamp(cvRound(px), 0, src_cols - 1);
//          const _Tpw ww = *((const _Tpw *)(wmap_base + wy_idx * wmap_stride) + wx_idx);
//          if ( !(ww > 0) ) {
//            continue;
//          }
//          if constexpr ( std::is_integral_v<_Tpw> ) {
//            w = 1.0f;
//          }
//          else {
//            w = ww;
//          }
//        }
//
//        // 2x2 neighborhood: sx0 <= px < sx1, sy0 <= py < sy1
//        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
//        const int sx1 = sx0 + 1, sy1 = sy0 + 1;
//
//        // acc footprint boundaries in CFA coordinates
//        const float ax0 = px - half_acc_x;
//        const float ax1 = px + half_acc_x;
//        const float ay0 = py - half_acc_y;
//        const float ay1 = py + half_acc_y;
//
//        // 1D intersections for 2x2 (given that sx0 <= px < sx1)
//        const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - ax0);
//        const float ov_x1 = std::max(0.0f, ax1 - std::max(ax0, sx1 - half_drop));
//
//        const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - ay0);
//        const float ov_y1 = std::max(0.0f, ay1 - std::max(ay0, sy1 - half_drop));
//
//        // Boundary validity masks
//        const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
//        const float wx1 = (sx1 < src_cols) ? ov_x1 : 0.0f;
//        const float wy0 = (sy0 >= 0) ? (ov_y0 * w) : 0.0f;
//        const float wy1 = (sy1 < src_rows) ? (ov_y1 * w) : 0.0f;
//
//        // Safe read indices
//        const int ix0 = std::max(sx0, 0);
//        const int iy0 = std::max(sy0, 0);
//        const int ix1 = std::min(sx1, src_cols - 1);
//        const int iy1 = std::min(sy1, src_rows - 1);
//
//        const _Tp * const srcp0 = (const _Tp *)(src_base + iy0 * src_stride);
//        const _Tp * const srcp1 = (const _Tp *)(src_base + iy1 * src_stride);
//
//        const float v00 = srcp0[ix0], v01 = srcp0[ix1];
//        const float v10 = srcp1[ix0], v11 = srcp1[ix1];
//
//        const float w00 = wy0 * wx0, w01 = wy0 * wx1;
//        const float w10 = wy1 * wx0, w11 = wy1 * wx1;
//
//        const int c00 = cfa_lut[sy0 & 1][sx0 & 1];
//        const int c01 = cfa_lut[sy0 & 1][sx1 & 1];
//        const int c10 = cfa_lut[sy1 & 1][sx0 & 1];
//        const int c11 = cfa_lut[sy1 & 1][sx1 & 1];
//
//        cv::Vec3f a = accp[x];
//        cv::Vec3f cnt = cntrp[x];
//
//        a[c00] += v00 * w00; cnt[c00] += w00;
//        a[c01] += v01 * w01; cnt[c01] += w01;
//        a[c10] += v10 * w10; cnt[c10] += w10;
//        a[c11] += v11 * w11; cnt[c11] += w11;
//
//        accp[x] = a;
//        cntrp[x] = cnt;
//      }
//    }
//  });
//
//  return true;
//}

template<class _Tp, class _Tpw, RmapMode _RmapMode>
static bool _rgb_drizzle_core(cv::InputArray _src_image,
    cv::InputOutputArray _acc, // cv::Mat3f
    cv::InputOutputArray _cntr, // cv::Mat1f
    const cv::Mat2f & rmap,
    cv::InputArray weights)
{
  const cv::Mat src_image = _src_image.getMat();
  const uint8_t * const src_base = src_image.ptr();
  const size_t src_stride = src_image.step;

  cv::Mat & acc = _acc.getMatRef();
  const uint8_t * const acc_base = acc.ptr();
  const size_t acc_stride = acc.step;

  cv::Mat & cntr = _cntr.getMatRef();
  const uint8_t * const cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  constexpr bool has_weights = !std::is_void_v<_Tpw>;
  const cv::Mat wmap = has_weights ? weights.getMat() : cv::Mat();
  const uint8_t * const wmap_base = has_weights ? wmap.ptr() : nullptr;
  const size_t wmap_stride = has_weights ? wmap.step : 0;

  const uint8_t * const rmap_base = (_RmapMode != RmapMode::Empty) ? rmap.ptr() : nullptr;
  const size_t rmap_stride = (_RmapMode != RmapMode::Empty) ? rmap.step : 0;

  const int src_channels = src_image.channels();
  const int src_rows = src_image.rows;
  const int src_cols = src_image.cols;
  const int acc_rows = acc.rows;
  const int acc_cols = acc.cols;

  const float step_x = float(src_cols) / float(acc_cols);
  const float step_y = float(src_rows) / float(acc_rows);

  // Ensure 1 pixel of the sensor array covers no more than 2x2 pixels of the CFA
  if ( step_x > 1.0f || step_y > 1.0f ) {
    CF_ERROR("Downscaling is not supported in 2x2 drizzle (step_x=%f, step_y=%f)", step_x, step_y);
    return false;
  }

  parallel_for(0, acc_rows, [=](const auto & range) {


    // Hardcoded pixfrac = 1 / scale = min(step_x, step_y)
    const float half_drop = 0.5f * std::max(1e-2f, std::min(step_x, step_y));
    const float half_acc_x = 0.5f * step_x;
    const float half_acc_y = 0.5f * step_y;
    const float offset_x = 0.5f * step_x - 0.5f;
    const float offset_y = 0.5f * step_y - 0.5f;
    const int cn = src_channels;

    for ( int y = rbegin(range); y < rend(range); ++y ) {
      float * __restrict accp = (float *)(acc_base + y * acc_stride);
      float * __restrict cntrp = (float *)(cntr_base + y * cntr_stride);
      const float rmap_y = y * step_y + offset_y;

      const cv::Vec2f * rmp0 = nullptr;
      const cv::Vec2f * rmp1 = nullptr;
      const cv::Vec2f * rmp2 = nullptr;
      const cv::Vec2f * rmp3 = nullptr;
      float rwy[4] = {0.0f, 1.0f, 0.0f, 0.0f};

      if constexpr ( _RmapMode == RmapMode::Direct ) {
        // remap.size() == acc.size()
        rmp1 = (const cv::Vec2f *)(rmap_base + y * rmap_stride);
      }
      else if constexpr ( _RmapMode == RmapMode::Scaled ) {
        // remap.size() == cfa_image.size()
        const float clamped_rmap_y = std::clamp(rmap_y, 0.0f, float(src_rows - 1));
        const int ry1 = cvFloor(clamped_rmap_y);
        const int ry0 = std::max(ry1 - 1, 0);
        const int ry2 = std::min(ry1 + 1, src_rows - 1);
        const int ry3 = std::min(ry1 + 2, src_rows - 1);

        catmull_rom_weights(clamped_rmap_y - ry1, rwy);

        rmp0 = (const cv::Vec2f *)(rmap_base + ry0 * rmap_stride);
        rmp1 = (const cv::Vec2f *)(rmap_base + ry1 * rmap_stride);
        rmp2 = (const cv::Vec2f *)(rmap_base + ry2 * rmap_stride);
        rmp3 = (const cv::Vec2f *)(rmap_base + ry3 * rmap_stride);
      }

      for ( int x = 0; x < acc_cols; ++x, accp += cn, ++cntrp ) {
        const float rmap_x = x * step_x + offset_x;

        float px, py;
        if constexpr ( _RmapMode == RmapMode::Empty ) {
          // no remap
          px = rmap_x;
          py = rmap_y;
        }
        else if constexpr ( _RmapMode == RmapMode::Direct ) {
          // remap.size() == acc.size()
          const cv::Vec2f p = rmp1[x];
          px = p[0];
          py = p[1];
        }
        else { // RmapMode::Scaled (Bicubic Catmull-Rom)
          // remap.size() == cfa_image.size()
          const float rx = std::clamp(rmap_x, 0.0f, float(src_cols - 1));
          const int rx1 = cvFloor(rx);
          const int rx0 = std::max(rx1 - 1, 0);
          const int rx2 = std::min(rx1 + 1, src_cols - 1);
          const int rx3 = std::min(rx1 + 2, src_cols - 1);

          float rwx[4];
          catmull_rom_weights(rx - rx1, rwx);

          const cv::Vec2f row0 = rmp0[rx0] * rwx[0] + rmp0[rx1] * rwx[1] + rmp0[rx2] * rwx[2] + rmp0[rx3] * rwx[3];
          const cv::Vec2f row1 = rmp1[rx0] * rwx[0] + rmp1[rx1] * rwx[1] + rmp1[rx2] * rwx[2] + rmp1[rx3] * rwx[3];
          const cv::Vec2f row2 = rmp2[rx0] * rwx[0] + rmp2[rx1] * rwx[1] + rmp2[rx2] * rwx[2] + rmp2[rx3] * rwx[3];
          const cv::Vec2f row3 = rmp3[rx0] * rwx[0] + rmp3[rx1] * rwx[1] + rmp3[rx2] * rwx[2] + rmp3[rx3] * rwx[3];
          const cv::Vec2f p = row0 * rwy[0] + row1 * rwy[1] + row2 * rwy[2] + row3 * rwy[3];
          px = p[0];
          py = p[1];
        }

        const bool in_bounds = (px >= -0.5f) & (px <= src_cols - 0.5f) & (py >= -0.5f) & (py <= src_rows - 0.5f);

        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
        const int sx1 = sx0 + 1, sy1 = sy0 + 1;

        const int ix0 = std::clamp(sx0, 0, src_cols - 1);
        const int iy0 = std::clamp(sy0, 0, src_rows - 1);
        const int ix1 = std::clamp(sx1, 0, src_cols - 1);
        const int iy1 = std::clamp(sy1, 0, src_rows - 1);


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

        float w00, w01, w10, w11;

        if constexpr (!has_weights ) {
          w00 = wy0 * wx0;
          w01 = wy0 * wx1;
          w10 = wy1 * wx0;
          w11 = wy1 * wx1;
        }
        else {
          const _Tpw * const m0 = (const _Tpw * )(wmap_base + iy0 * wmap_stride);
          const _Tpw * const m1 = (const _Tpw * )(wmap_base + iy1 * wmap_stride);
          w00 = wy0 * wx0 * m0[ix0];
          w01 = wy0 * wx1 * m0[ix1];
          w10 = wy1 * wx0 * m1[ix0];
          w11 = wy1 * wx1 * m1[ix1];
        }

        const _Tp * const src0 = (const _Tp *)(src_base + iy0 * src_stride);
        const _Tp * const src1 = (const _Tp *)(src_base + iy1 * src_stride);
        const _Tp * const s00 = src0 + ix0 * cn;
        const _Tp * const s01 = src0 + ix1 * cn;
        const _Tp * const s10 = src1 + ix0 * cn;
        const _Tp * const s11 = src1 + ix1 * cn;

        const float wsum = w00 + w01 + w10 + w11;
        *cntrp += wsum;
        for ( int c = 0; c < cn; ++c ) {
          const float val = s00[c] * w00 + s01[c] * w01 + s10[c] * w10 + s11[c] * w11;
          accp[c] += val;
        }
      }
    }
  });

  return true;
}

template<class _Tp, class _Tpw>
static bool _rgb_drizzle_dispatch_rmap(cv::InputArray src,
    cv::InputOutputArray acc, cv::InputOutputArray  cntr,
    const cv::Mat2f & rmap,
    cv::InputArray weights)
{
  if ( rmap.empty() ) {
    return _rgb_drizzle_core<_Tp, _Tpw, RmapMode::Empty>(src, acc, cntr, rmap, weights);
  }

  if ( rmap.size() == acc.size() ) {
    return _rgb_drizzle_core<_Tp, _Tpw, RmapMode::Direct>(src, acc, cntr, rmap, weights);
  }

  if ( rmap.size() == src.size() ) {
    return _rgb_drizzle_core<_Tp, _Tpw, RmapMode::Scaled>(src, acc, cntr, rmap, weights);
  }

  CF_ERROR("Invalid remap size (%dx%d) for src size (%dx%d) and acc size (%dx%d)",
      rmap.cols, rmap.rows,
      src.cols(), src.rows(),
      acc.cols(), acc.rows());
  return false;
}

template<class _Tp>
static bool _rgb_drizzle_dispatch_weights(cv::InputArray src,
    cv::InputOutputArray acc, cv::InputOutputArray cntr,
    const cv::Mat2f & rmap,
    cv::InputArray weights)
{
  if ( weights.empty() ) {
    return _rgb_drizzle_dispatch_rmap<_Tp, void>(src, acc, cntr, rmap, weights);
  }

  if ( weights.size() != src.size() ) {
    CF_ERROR("Invalid weights size (%dx%d) for src size (%dx%d)", weights.cols(), weights.rows(),
        src.cols(), src.rows());
    return false;
  }

  switch ( weights.type() ) {
    case CV_8UC1:
      return _rgb_drizzle_dispatch_rmap<_Tp, uint8_t>(src, acc, cntr, rmap, weights);
    case CV_32FC1:
      return _rgb_drizzle_dispatch_rmap<_Tp, float>(src, acc, cntr, rmap, weights);
    default:
      break;
  }

  CF_ERROR("Not supported weights type=%d in cfa_accumulate (expected CV_8UC1 or CV_32FC1)", weights.type());
  return false;
}

static bool rgb_drizzle_accumulate(cv::InputArray src,
    cv::InputOutputArray acc, cv::InputOutputArray cntr,
    const cv::Mat2f & rmap,
    cv::InputArray weights)
{
  INSTRUMENT_REGION("");

  if ( src.empty() || src.channels() != acc.channels() ) {
    CF_ERROR("src channels==%d not match to acc.channels=%d", src.channels(), acc.channels());
    return false;
  }

  if ( acc.empty() || cntr.empty() || acc.size() != cntr.size() ) {
    CF_ERROR("Invalid or mismatched accumulator (%dx%d) and counter (%dx%d) sizes",
        acc.cols(), acc.rows(), cntr.cols(), cntr.rows());
    return false;
  }

  CV_DISPATCH(src.depth(), _rgb_drizzle_dispatch_weights,
      src, acc, cntr, rmap, weights);

  CF_ERROR("APP BUG: BAD srcImage.depth()=%d", src.depth());
  return false;
}

bool c_rgb_drizzle::add(cv::InputArray src, cv::InputArray mask)
{
  const cv::Size src_size = src.size();
  const int src_channels = src.channels();
  const int msk_channels = mask.channels();

  cv::Size dst_size;

  switch (_scale) {
    case DRIZZLE_SCALE_10x:
      dst_size = src_size;
      break;
    case DRIZZLE_SCALE_15x:
      dst_size.width = src_size.width * 3 / 2;
      dst_size.height = src_size.height * 3 / 2;
      break;
    case DRIZZLE_SCALE_20x:
      dst_size.width = src_size.width * 2;
      dst_size.height = src_size.height * 2;
      break;
    case DRIZZLE_SCALE_30x:
      dst_size.width = src_size.width * 3;
      dst_size.height = src_size.height * 3;
      break;
    case DRIZZLE_SCALE_40x:
      dst_size.width = src_size.width * 4;
      dst_size.height = src_size.height * 4;
      break;

    default:
      CF_ERROR("Bad or not supported scale = %d requested", _scale);
      return false;
  }

  if ( true ) {
    std::scoped_lock lock(_mtx);

    if( _accumulated_frames < 1 ) {
      _accumulator.create(dst_size, CV_MAKETYPE(CV_32F, src_channels));
      _counter.create(dst_size, CV_MAKETYPE(CV_32F, msk_channels));
      _accumulator.setTo(cv::Scalar::all(0));
      _counter.setTo(cv::Scalar::all(0));
      _accumulated_frames = 0;
    }
    else if( _accumulator.size() != dst_size ) {
      CF_ERROR("Expected destination size (%dx%d) does not match accumulator size (%dx%d)",
          dst_size.width, dst_size.height, _accumulator.cols, _accumulator.rows);
      return false;
    }
  }

  if( !rgb_drizzle_accumulate(src, _accumulator, _counter, _rmap, mask) ) {
    CF_ERROR("cfa_accumulate() fails");
    return false;
  }

  ++_accumulated_frames;
  return true;
}

template<class _Tp, bool _NeedAvg, bool _NeedMask>
static void _compute_rgb_drizzle_output_impl(cv::InputArray _acc, cv::InputArray _cntr,
    cv::OutputArray avg, cv::OutputArray mask, float scale)
{
  constexpr float eps = 1e-5f;

  const cv::Mat acc = _acc.getMat();
  const cv::Mat cntr = _cntr.getMat();

  const cv::Size size = acc.size();
  const int acc_channels = acc.channels();
  const int cntr_channels = cntr.channels();

  cv::Mat dst_avg;
  if constexpr ( _NeedAvg ) {
    avg.create(size, CV_MAKETYPE(cv::DataType<_Tp>::depth, acc_channels));
    dst_avg = avg.getMatRef();
  }

  cv::Mat1b dst_mask;
  if constexpr ( _NeedMask ) {
    mask.create(size, CV_8UC1);
    dst_mask = mask.getMat();
  }

  const uint8_t * const acc_base = acc.ptr();
  const size_t acc_stride = acc.step;
  const uint8_t * const cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  uint8_t * const avg_base = _NeedAvg ? dst_avg.ptr() : nullptr;
  const size_t avg_stride = _NeedAvg ? dst_avg.step : 0;
  uint8_t * const msk_base = _NeedMask ? dst_mask.ptr() : nullptr;
  const size_t msk_stride = _NeedMask ? dst_mask.step : 0;

  parallel_for(0, size.height, [=](const auto & range) {
    for ( int y = rbegin(range); y < rend(range); ++y ) {

      const float * accp = (const float *)(acc_base + y * acc_stride);
      const float * cntrp = (const float*)(cntr_base + y * cntr_stride);
      float * __restrict avgp = _NeedAvg ? (float *)(avg_base + y * avg_stride) : nullptr;
      uint8_t * __restrict mskp = _NeedMask ? (msk_base + y * msk_stride) : nullptr;

      if constexpr ( _NeedAvg ) {
        if ( acc_channels == cntr_channels ) {
          const int xmax = size.width * acc_channels;
          for ( int x = 0; x < xmax; ++x ) {
            const float w = cntrp[x];
            avgp[x] = w > eps ? accp[x] * scale / w : 0;
          }
        }
        else {
          const int xmax = size.width;
          for ( int x = 0; x < xmax; ++x, accp += acc_channels, avgp += acc_channels ) {
            const float w = cntrp[x];
            for ( int c = 0; c < acc_channels; ++c ) {
              avgp[c] = w > eps ? accp[c] * scale / w : 0;
            }
          }
        }
      }

      if constexpr ( _NeedMask ) {
        const int xmax = size.width;
        if ( cntr_channels == 1 ) {
          for ( int x = 0; x < xmax; ++x ) {
            mskp[x] = cntrp[x] > 0 ? 255 : 0;
          }
        }
        else {
          for ( int x = 0; x < xmax; ++x, cntrp += cntr_channels ) {
            float w = cntrp[0];
            for ( int c = 1; c < cntr_channels; ++c ) {
              w = std::max(w, cntrp[c]);
            }
            mskp[x] = w > 0 ? 255 : 0;
          }
        }
      }
    }
  });
}

template<class _Tp>
static bool _compute_rgb_drizzle_output(cv::InputArray acc, cv::InputArray cntr,
    cv::OutputArray avg, cv::OutputArray mask, float scale)
{
  const bool need_avg = avg.needed();
  const bool need_mask = mask.needed();

  if ( need_avg && need_mask ) {
    _compute_rgb_drizzle_output_impl<_Tp, true, true>(acc, cntr, avg, mask, scale);
  }
  else if ( need_avg ) {
    _compute_rgb_drizzle_output_impl<_Tp, true, false>(acc, cntr, avg, mask, scale);
  }
  else if ( need_mask ) {
    _compute_rgb_drizzle_output_impl<_Tp, false, true>(acc, cntr, avg, mask, scale);
  }

  return true;
}

bool c_rgb_drizzle::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  if ( !avg.needed() && !mask.needed() ) {
    return true;
  }

  cv::Mat acc, cntr;
  int accumulated_frames = 0;

  synchronized([&]() {
    acc = _accumulator;
    cntr = _counter;
    accumulated_frames = _accumulated_frames;
  });

  if ( accumulated_frames < 1 ) {
    CF_ERROR("no frames");
    return false;
  }

  if ( avg.fixedType() ) {
    ddepth = avg.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = CV_32F;
  }

  CV_DISPATCH(ddepth, _compute_rgb_drizzle_output, acc, cntr, avg, mask, float(dscale));
  CF_ERROR("APP BUG: BAD ddepth=%d encountered in c_bayer_drizzle::compute", ddepth);
  return false;
}





