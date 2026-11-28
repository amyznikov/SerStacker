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
// Precompute weights and the boundaries of the safe central zone [x_begin, x_end) on X
struct alignas(16) XTap {
  int x0;
  float wx0;
  float wx1;
  float wx2;
};

// weights for the Catmull-Rom bicubic spline (a = -0.5) for t in [0, 1]
static ALWAYS_INLINE
void catmull_rom_weights(float t, float w[4])
{
  const float t2 = t * t;
  const float t3 = t * t * t;
  w[0] = -0.5f * t + t2 - 0.5f * t3;
  w[1] = 1.0f - 2.5f * t2 + 1.5f * t3;
  w[2] = 0.5f * t + 2.0f * t2 - 1.5f * t3;
  w[3] = -0.5f * t2 + 0.5f * t3;
}

static void generate_xtabs(const cv::Size & srcSize, const cv::Size & dstSize, std::vector<XTap> & xtabs,
    int & out_x_begin, int & out_x_end)
{
  int x_begin = 0;
  int x_end = dstSize.width;

  const int max_x = srcSize.width - 1;
  const float scale_x = float(srcSize.width) / float(dstSize.width);

  xtabs.resize(dstSize.width);

  for ( int x = 0; x < dstSize.width; ++x ) {
    const float fx = (float(x) + 0.5f) * scale_x - 0.5f;
    const int xc = cvFloor(fx + 0.5f);
    const float dx = fx - xc; // [-0.5, 0.5]

    if ( xc - 1 < 0 ) {
      x_begin = x + 1;
    }
    if ( xc + 1 > max_x && x < x_end ) {
      x_end = x;
    }

    XTap & tap = xtabs[x];
    tap.x0 = xc - 1;
    tap.wx0 = 0.5f * (0.5f - dx) * (0.5f - dx);
    tap.wx1 = 0.75f - dx * dx;
    tap.wx2 = 0.5f * (0.5f + dx) * (0.5f + dx);
  }

  if ( x_begin > x_end ) {
    x_begin = x_end;
  }
}

template<class _Tp, class _Tpw>
static ALWAYS_INLINE
void accumualate_pixel(float px, float py,
    const int src_cols, const int src_rows,
    const float half_drop, const float half_acc_x, const float half_acc_y,
    const uint8_t * src_base, const size_t src_stride,
    const uint8_t * wmap_base, const size_t wmap_stride,
    float * __restrict accp, float * __restrict cntrp,
    const int src_channels, const int wmap_channels)
{
  constexpr bool has_weights = !std::is_void_v<_Tpw>;

  const bool in_bounds = (px >= -0.5f) & (px <= src_cols - 0.5f) & (py >= -0.5f) & (py <= src_rows - 0.5f);
  const float valid_mask = in_bounds ? 1.0f : 0.0f;

  const int sx0 = cvFloor(px), sy0 = cvFloor(py);
  const int sx1 = sx0 + 1, sy1 = sy0 + 1;
  const int ix0 = std::clamp(sx0, 0, src_cols - 1);
  const int iy0 = std::clamp(sy0, 0, src_rows - 1);
  const int ix1 = std::clamp(sx1, 0, src_cols - 1);
  const int iy1 = std::clamp(sy1, 0, src_rows - 1);

  const _Tp * const src0 = (const _Tp*) (src_base + iy0 * src_stride);
  const _Tp * const src1 = (const _Tp*) (src_base + iy1 * src_stride);
  const _Tp * const s00 = src0 + ix0 * src_channels;
  const _Tp * const s01 = src0 + ix1 * src_channels;
  const _Tp * const s10 = src1 + ix0 * src_channels;
  const _Tp * const s11 = src1 + ix1 * src_channels;

  const float ax0 = px - half_acc_x, ax1 = px + half_acc_x;
  const float ay0 = py - half_acc_y, ay1 = py + half_acc_y;
  const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - std::max(ax0, sx0 - half_drop));
  const float ov_x1 = std::max(0.0f, std::min(ax1, sx1 + half_drop) - std::max(ax0, sx1 - half_drop));
  const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - std::max(ay0, sy0 - half_drop));
  const float ov_y1 = std::max(0.0f, std::min(ay1, sy1 + half_drop) - std::max(ay0, sy1 - half_drop));
  const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
  const float wx1 = (sx1 < src_cols) ? ov_x1 : 0.0f;
  const float wy0 = (sy0 >= 0) ? (ov_y0 * valid_mask) : 0.0f;
  const float wy1 = (sy1 < src_rows) ? (ov_y1 * valid_mask) : 0.0f;

  if constexpr ( !has_weights ) {
    const float w00 = wy0 * wx0, w01 = wy0 * wx1;
    const float w10 = wy1 * wx0, w11 = wy1 * wx1;
    const float wsum = w00 + w01 + w10 + w11;

    if( wmap_channels == 1 ) {
      *cntrp += wsum;
      for( int c = 0; c < src_channels; ++c ) {
        const float val = s00[c] * w00 + s01[c] * w01 + s10[c] * w10 + s11[c] * w11;
        accp[c] += val;
      }
    }
    else {
      for( int c = 0; c < src_channels; ++c ) {
        const float val = s00[c] * w00 + s01[c] * w01 + s10[c] * w10 + s11[c] * w11;
        accp[c] += val;
        cntrp[c] += wsum;
      }
    }
  }
  else { // wmap_channels == cntr_channels always when has_weights
    const _Tpw * const m00 = (const _Tpw*) (wmap_base + iy0 * wmap_stride) + ix0 * wmap_channels;
    const _Tpw * const m01 = (const _Tpw*) (wmap_base + iy0 * wmap_stride) + ix1 * wmap_channels;
    const _Tpw * const m10 = (const _Tpw*) (wmap_base + iy1 * wmap_stride) + ix0 * wmap_channels;
    const _Tpw * const m11 = (const _Tpw*) (wmap_base + iy1 * wmap_stride) + ix1 * wmap_channels;

    if( wmap_channels == 1 ) {
      const float w00 = wy0 * wx0 * m00[0];
      const float w01 = wy0 * wx1 * m01[0];
      const float w10 = wy1 * wx0 * m10[0];
      const float w11 = wy1 * wx1 * m11[0];
      const float wsum = w00 + w01 + w10 + w11;
      *cntrp += wsum;
      for( int c = 0; c < src_channels; ++c ) {
        const float val = s00[c] * w00 + s01[c] * w01 + s10[c] * w10 + s11[c] * w11;
        accp[c] += val;
      }
    }
    else {
      for( int c = 0; c < src_channels; ++c ) {
        const float w00 = wy0 * wx0 * m00[c];
        const float w01 = wy0 * wx1 * m01[c];
        const float w10 = wy1 * wx0 * m10[c];
        const float w11 = wy1 * wx1 * m11[c];
        const float wsum = w00 + w01 + w10 + w11;
        const float val = s00[c] * w00 + s01[c] * w01 + s10[c] * w10 + s11[c] * w11;
        cntrp[c] += wsum;
        accp[c] += val;
      }
    }
  }
}

} // namespace



template<class _Tp, class _Tpw>
static bool _rgb_drizzle_core(cv::InputArray _src_image,
    cv::InputOutputArray _acc, // cv::Mat_<Tp, acc_channels>
    cv::InputOutputArray _cntr, // cv::Mat_<Tp, cntr_channels>
    const cv::Mat2f & rmap,
    cv::InputArray weights)
{
  constexpr bool has_weights = !std::is_void_v<_Tpw>;

  if ( _src_image.empty() ) {
    CF_ERROR("Bad _src_image: empty");
    return false;
  }

  if ( _acc.empty() ) {
    CF_ERROR("Bad _acc: not initialized");
    return false;
  }

  if ( _cntr.empty() ) {
    CF_ERROR("Bad _cntr: not initialized");
    return false;
  }

  if ( !rmap.empty() && rmap.size() != _src_image.size()) {
    CF_ERROR("Bad rmap size: %dx%d. Must be %dx%d", rmap.cols, rmap.rows,
        _src_image.cols(), _src_image.rows());
    return false;
  }

  if constexpr (has_weights) {
    if ( weights.size() != _src_image.size()) {
      CF_ERROR("Bad weights size: %dx%d. Must be %dx%d", weights.cols(), weights.rows(),
          _src_image.cols(), _src_image.rows());
      return false;
    }
  }

  const int src_channels = _src_image.channels();
  const int acc_channels = _acc.channels();
  const int cntr_channels = _cntr.channels();
  const int wmap_channels = has_weights ? weights.channels() :  1;

  if ( src_channels != acc_channels ) {
    CF_ERROR("Bad src image channels: %d. Must be acc_channels=%d", src_channels, acc_channels);
    return false;
  }

  if ( cntr_channels != 1 && cntr_channels != acc_channels ) {
    CF_ERROR("Bad cntr channels: %d. Must be 1 or acc_channels=%d", cntr_channels, acc_channels);
    return false;
  }

  if constexpr (has_weights) {
    if ( wmap_channels != 1 && wmap_channels != src_channels ) {
      CF_ERROR("Bad wmap channels: %d. Must be 1 or src_channels=%d", wmap_channels, src_channels);
      return false;
    }
  }

  const cv::Mat src = _src_image.getMat();
  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  const cv::Mat wmap = has_weights ? weights.getMat() : cv::Mat();
  const uint8_t * const wmap_base = has_weights ? wmap.ptr() : nullptr;
  const size_t wmap_stride = has_weights ? wmap.step : 0;

  cv::Mat & acc = _acc.getMatRef();
  const uint8_t * const acc_base = acc.ptr();
  const size_t acc_stride = acc.step;

  cv::Mat & cntr = _cntr.getMatRef();
  const uint8_t * const cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  const uint8_t * const rmap_base = rmap.empty() ? nullptr : rmap.ptr();
  const size_t rmap_stride = rmap.empty() ? 0 : rmap.step;

  const cv::Size srcSize = src.size();
  const cv::Size dstSize = acc.size();
  const int src_rows = src.rows;
  const int src_cols = src.cols;
  const int acc_rows = acc.rows;
  const int acc_cols = acc.cols;
  const float step_x = float(src_cols) / float(acc_cols);
  const float step_y = float(src_rows) / float(acc_rows);

  // Ensure 1 pixel of the sensor array covers no more than 2x2 pixels of the CFA
  if ( step_x > 1.0f || step_y > 1.0f ) {
    CF_ERROR("Downscaling is not supported in 2x2 drizzle (step_x=%f, step_y=%f)", step_x, step_y);
    return false;
  }

//  // Precompute weights and the boundaries of the safe central zone [x_begin, x_end) on X
//  std::vector<XTap> xtabs(dstSize.width);
//  int x_begin = 0, x_end = dstSize.width;
//  generate_xtabs(srcSize, dstSize, xtabs, x_begin, x_end);
//
//  const XTap * const __restrict xtabs_ptr = xtabs.data();
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

      if ( rmap_base ) { // remap.size() == src_image.size()
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
        if ( !rmap_base ) { // no remap
          px = rmap_x;
          py = rmap_y;
        }
        else { // Bicubic Catmull-Rom
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

        accumualate_pixel<_Tp, _Tpw>(px, py, src_cols, src_rows,
            half_drop, half_acc_x, half_acc_y,
            src_base, src_stride,
            wmap_base, wmap_stride,
            accp, cntrp,
            src_channels, wmap_channels);


//        const bool in_bounds = (px >= -0.5f) & (px <= src_cols - 0.5f) & (py >= -0.5f) & (py <= src_rows - 0.5f);
//
//        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
//        const int sx1 = sx0 + 1, sy1 = sy0 + 1;
//
//        const int ix0 = std::clamp(sx0, 0, src_cols - 1);
//        const int iy0 = std::clamp(sy0, 0, src_rows - 1);
//        const int ix1 = std::clamp(sx1, 0, src_cols - 1);
//        const int iy1 = std::clamp(sy1, 0, src_rows - 1);
//
//        const float ax0 = px - half_acc_x;
//        const float ax1 = px + half_acc_x;
//        const float ay0 = py - half_acc_y;
//        const float ay1 = py + half_acc_y;
//
//        const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - std::max(ax0, sx0 - half_drop));
//        const float ov_x1 = std::max(0.0f, std::min(ax1, sx1 + half_drop) - std::max(ax0, sx1 - half_drop));
//        const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - std::max(ay0, sy0 - half_drop));
//        const float ov_y1 = std::max(0.0f, std::min(ay1, sy1 + half_drop) - std::max(ay0, sy1 - half_drop));
//
//        const float valid_mask = in_bounds ? 1.0f : 0.0f;
//        const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
//        const float wx1 = (sx1 < src_cols) ? ov_x1 : 0.0f;
//        const float wy0 = (sy0 >= 0) ? (ov_y0 * valid_mask) : 0.0f;
//        const float wy1 = (sy1 < src_rows) ? (ov_y1 * valid_mask) : 0.0f;
//
//        float w00, w01, w10, w11;
//
//        if constexpr (!has_weights ) {
//          w00 = wy0 * wx0;
//          w01 = wy0 * wx1;
//          w10 = wy1 * wx0;
//          w11 = wy1 * wx1;
//        }
//        else {
//          const _Tpw * const m0 = (const _Tpw * )(wmap_base + iy0 * wmap_stride);
//          const _Tpw * const m1 = (const _Tpw * )(wmap_base + iy1 * wmap_stride);
//          w00 = wy0 * wx0 * m0[ix0];
//          w01 = wy0 * wx1 * m0[ix1];
//          w10 = wy1 * wx0 * m1[ix0];
//          w11 = wy1 * wx1 * m1[ix1];
//        }
//
//        const _Tp * const src0 = (const _Tp *)(src_base + iy0 * src_stride);
//        const _Tp * const src1 = (const _Tp *)(src_base + iy1 * src_stride);
//        const _Tp * const s00 = src0 + ix0 * cn;
//        const _Tp * const s01 = src0 + ix1 * cn;
//        const _Tp * const s10 = src1 + ix0 * cn;
//        const _Tp * const s11 = src1 + ix1 * cn;
//
//        const float wsum = w00 + w01 + w10 + w11;
//        *cntrp += wsum;
//        for ( int c = 0; c < cn; ++c ) {
//          const float val = s00[c] * w00 + s01[c] * w01 + s10[c] * w10 + s11[c] * w11;
//          accp[c] += val;
//        }
      }
    }
  });

  return true;
}

template<class _Tp>
static bool _rgb_drizzle_dispatch_weights(cv::InputArray src,
    cv::InputOutputArray acc, cv::InputOutputArray cntr,
    const cv::Mat2f & rmap,
    cv::InputArray weights)
{
  if ( weights.empty() ) {
    return _rgb_drizzle_core<_Tp, void>(src, acc, cntr, rmap, weights);
  }

  switch ( weights.type() ) {
    case CV_8UC1:
      return _rgb_drizzle_core<_Tp, uint8_t>(src, acc, cntr, rmap, weights);
    case CV_32FC1:
      return _rgb_drizzle_core<_Tp, float>(src, acc, cntr, rmap, weights);
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

  if ( !rmap.empty() && rmap.size() != src.size() ) {
    CF_ERROR("Invalid rmap size (%dx%d) for src size (%dx%d)", rmap.cols, rmap.rows,
        src.cols(), src.rows());
    return false;
  }

  if ( !weights.empty() && weights.size() != src.size() ) {
    CF_ERROR("Invalid weights size (%dx%d) for src size (%dx%d)", weights.cols(), weights.rows(),
        src.cols(), src.rows());
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





