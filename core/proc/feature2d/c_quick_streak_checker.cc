/*
 * c_quick_streak_checker.cc
 *
 *  Created on: Nov 17, 2026
 *      Author: amyznikov
 */

#include "c_quick_streak_checker.h"
#include <core/proc/morphse.h>
#include <core/proc/median.h>
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

// Packing a node into a single 64-bit register:
// [63..48] = len (uint16_t)
// [31..16] = sy  (uint16_t)
// [15..0]  = sx  (uint16_t)
static constexpr uint64_t LEN_SHIFT = 48;
static constexpr uint64_t ONE_STEP  = (1ULL << LEN_SHIFT);

static ALWAYS_INLINE
uint64_t pack_node(uint32_t len, uint32_t sx, uint32_t sy)
{
  return (uint64_t(len) << LEN_SHIFT) | (uint64_t(sy)  << 16) | uint64_t(sx);
}

static ALWAYS_INLINE
uint32_t node_len(uint64_t node)
{
  return static_cast<uint32_t>(node >> LEN_SHIFT);
}

static ALWAYS_INLINE
uint32_t node_sx(uint64_t node)
{
  return static_cast<uint32_t>(node & 0xFFFFU);
}

static ALWAYS_INLINE
uint32_t node_sy(uint64_t node)
{
  return static_cast<uint32_t>((node >> 16) & 0xFFFFU);
}

static ALWAYS_INLINE
uint64_t decay_packed(uint64_t node, uint64_t penalty_step)
{
  return (node > penalty_step) ? (node - penalty_step) : 0ULL;
}

static ALWAYS_INLINE
bool is_thin_streak(const uint8_t* mask_base, size_t mask_stride,
    int width, int height, int x1, int y1, int x2, int y2,
    int64_t chord_sq)
{
  const float inv_len = 3.0f / std::sqrt(static_cast<float>(chord_sq));
  const int dx = x2 - x1;
  const int dy = y2 - y1;

  const int nx = static_cast<int>(std::lround(-dy * inv_len));
  const int ny = static_cast<int>(std::lround( dx * inv_len));

  int thick_hits = 0;

  // Control point 1 (1/3 of the length)
  if ( true ) {
    const int px = x1 + dx / 3;
    const int py = y1 + dy / 3;
    const int lx = px + nx, ly = py + ny;
    const int rx = px - nx, ry = py - ny;
    if ((unsigned)lx < (unsigned)width && (unsigned)ly < (unsigned)height &&
        (unsigned)rx < (unsigned)width && (unsigned)ry < (unsigned)height) {
      if (mask_base[ly * mask_stride + lx] && mask_base[ry * mask_stride + rx]) {
        ++thick_hits;
      }
    }
  }

  // Checkpoint 2 (2/3 length)
  if ( true ) {
    const int px = x1 + (2 * dx) / 3;
    const int py = y1 + (2 * dy) / 3;
    const int lx = px + nx, ly = py + ny;
    const int rx = px - nx, ry = py - ny;
    if ((unsigned)lx < (unsigned)width && (unsigned)ly < (unsigned)height &&
        (unsigned)rx < (unsigned)width && (unsigned)ry < (unsigned)height) {
      if (mask_base[ly * mask_stride + lx] && mask_base[ry * mask_stride + rx]) {
        ++thick_hits;
      }
    }
  }

  return thick_hits < 2;
}

static ALWAYS_INLINE
bool check_hit_packed(uint64_t best, int x, int y, uint32_t target_len, int64_t min_chord_sq,
  const uint8_t* mask_base, size_t mask_stride, int width, int height,
  uint64_t& out_node,
  cv::Vec4i* out_track)
{
  if (best == 0ULL) {
    out_node = pack_node(1U, static_cast<uint32_t>(x), static_cast<uint32_t>(y));
    return false;
  }

  const uint64_t next_node = best + ONE_STEP;
  if (node_len(next_node) >= target_len) {
    const int sx = int(node_sx(best));
    const int sy = int(node_sy(best));
    const int64_t dx = int64_t(x) - sx;
    const int64_t dy = int64_t(y) - sy;
    const int64_t chord_sq = dx * dx + dy * dy;

    if (chord_sq >= min_chord_sq &&
        is_thin_streak(mask_base, mask_stride, width, height, sx, sy, x, y, chord_sq)) {
      if (out_track) {
        *out_track = cv::Vec4i(sx, sy, x, y);
      }
      return true;
    }
  }

  out_node = next_node;
  return false;
}

} // namespace

/**
* @brief Quick single-pass checker for the presence of at least one linear streak within a binary mask.
* @param _mask  Input binary mask (CV_8UC1; non-zero pixels are considered signal).
* @param minTrackLength Minimum length of a continuous, directional chain (in pixels).
* @return true if at least one track is detected
*/
static bool quick_streak_checker(cv::InputArray _mask, std::vector<uint64_t>& scratchBuf,
    int minTrackLength, float minStraightness,
    cv::Vec4i* outTrack = nullptr)
{
  INSTRUMENT_REGION("");

  if (outTrack) {
    *outTrack = cv::Vec4i(0, 0, 0, 0);
  }

  if (_mask.empty() || _mask.type() != CV_8UC1) {
    CF_ERROR("Bad input image: single-channel CV_8UC1 image expected");
    return false;
  }

  const int width = _mask.cols();
  const int height = _mask.rows();
  if (width < 3 || height < 3 || minTrackLength < 3 || width > 65530 || height > 65530) {
    return false;
  }

  const cv::Mat mask = _mask.getMat();
  const uint8_t* mask_base = mask.ptr();
  const size_t mask_stride = mask.step;

  // 4 uint64_t channels per node: d0 (↘), d1 (↓), d2 (→), d3 (↙)
  struct PackedCounters
  {
    uint64_t d0;
    uint64_t d1;
    uint64_t d2;
    uint64_t d3;
  };
  static_assert(sizeof(PackedCounters) == 4 * sizeof(uint64_t), "Unexpected padding");

  const size_t row_elems = static_cast<size_t>(width + 2);
  const size_t total_u64 = 2 * row_elems * 4;
  if (scratchBuf.size() < total_u64) {
    scratchBuf.resize(total_u64);
  }
  std::memset(scratchBuf.data(), 0, total_u64 * sizeof(uint64_t));

  PackedCounters* prev_row = reinterpret_cast<PackedCounters*>(scratchBuf.data());
  PackedCounters* curr_row = prev_row + row_elems;

  const uint32_t target_len = uint32_t(std::min(minTrackLength, 65530));
  const int64_t min_chord_sq = int64_t(std::lround((minStraightness * minTrackLength) * (minStraightness * minTrackLength)));
  constexpr uint64_t PENALTY_STEP = (2ULL << LEN_SHIFT);

  for (int y = 0; y < height; ++y) {
    const uint8_t* msk = mask_base + y * mask_stride;

    *curr_row = { 0ULL, 0ULL, 0ULL, 0ULL };
    curr_row[width + 1] = { 0ULL, 0ULL, 0ULL, 0ULL };

    // Pass from left to right: d0 (↘), d1 (↓), d2 (→)
    for (int x = 0; x < width; ++x) {
      const int xx = x + 1;
      const PackedCounters& up_left = prev_row[xx - 1];
      const PackedCounters& up      = prev_row[xx];
      const PackedCounters& left    = curr_row[xx - 1];

      if (!msk[x]) {
        curr_row[xx].d0 = decay_packed(up_left.d0, PENALTY_STEP);
        curr_row[xx].d1 = decay_packed(up.d1,      PENALTY_STEP);
        curr_row[xx].d2 = decay_packed(left.d2,    PENALTY_STEP);
      }
      else {
        const uint64_t b0 = std::max({ up_left.d0, up.d0, left.d0 });
        const uint64_t b1 = std::max({ up.d1, up_left.d1, prev_row[xx + 1].d1 });
        const uint64_t b2 = std::max(left.d2, up_left.d2);

        if (check_hit_packed(b0, x, y, target_len, min_chord_sq, mask_base, mask_stride, width, height, curr_row[xx].d0, outTrack) ||
            check_hit_packed(b1, x, y, target_len, min_chord_sq, mask_base, mask_stride, width, height, curr_row[xx].d1, outTrack) ||
            check_hit_packed(b2, x, y, target_len, min_chord_sq, mask_base, mask_stride, width, height, curr_row[xx].d2, outTrack)) {
          return true;
        }
      }
    }

    // Backward Pass from right to left: d3 (↙)
    for (int x = width - 1; x >= 0; --x) {
      const int xx = x + 1;
      const PackedCounters& up_right = prev_row[xx + 1];
      const PackedCounters& up       = prev_row[xx];
      const PackedCounters& right    = curr_row[xx + 1];

      if (!msk[x]) {
        curr_row[xx].d3 = decay_packed(up_right.d3, PENALTY_STEP);
      }
      else {
        const uint64_t b3 = std::max({ up_right.d3, up.d3, right.d3 });
        if (check_hit_packed(b3, x, y, target_len, min_chord_sq, mask_base, mask_stride, width, height, curr_row[xx].d3, outTrack)) {
          return true;
        }
      }
    }

    std::swap(prev_row, curr_row);
  }

  return false;
}

bool c_quick_streak_checker::check_for_streaks(const c_quick_streak_checker_options & opts,
    cv::InputArray _src, cv::InputArray _mask, cv::Vec4i* outTrack)
{
  INSTRUMENT_REGION("");
  cv::Mat src;

  if ( _src.channels() == 1 ) {
    src = _src.getMat();
  }
  else {
    INSTRUMENT_REGION("cvtColor");
    cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
  }

  /*
   * Pyramid down to specific level
   */
  if ( opts.pyramidLevel < 1 ) {
    cv::morphologyEx(src, _tophat, cv::MORPH_TOPHAT, morphse55(),
        cv::Point(-1,-1), 1,
        cv::BORDER_REPLICATE);
  }
  else {
    cv::pyrDown(src, _tophat);
    for( int l = 1; l < opts.pyramidLevel; ++l ) {
      cv::pyrDown(_tophat, _tophat);
    }
    cv::morphologyEx(_tophat, _tophat, cv::MORPH_TOPHAT, morphse55(),
        cv::Point(-1,-1), 1,
        cv::BORDER_REPLICATE);
  }

  _median = estimateMedianStdev(_tophat, &_sigma, 4, 4);
  _threshold = _median + opts.noiseSigmaScale * _sigma;
  cv::compare(_tophat, _threshold, _topmask, cv::CMP_GT);

  if ( !_mask.empty() ) {
    cv::Mat msk;
    if ( _mask.size() == _topmask.size() ) {
      if ( _mask.depth() == CV_8U ) {
        msk = _mask.getMat();
      }
      else {
        cv::compare(_mask, 0, msk, cv::CMP_GT);
      }
    }
    else {
      cv::resize(_mask, msk, _topmask.size(), 0, 0, cv::INTER_NEAREST);
      if ( msk.depth() != CV_8U ) {
        cv::compare(msk, 0, msk, cv::CMP_GT);
      }
    }

    cv::bitwise_and(msk, _topmask, _topmask);
  }

  return quick_streak_checker(_topmask, _scratchBuf, opts.minTrackLength, opts.minStraightness, outTrack);
}

bool serialize_quick_streak_checker_options(c_config_setting section, bool save,
    c_quick_streak_checker_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, pyramidLevel);
  SERIALIZE_OPTION(section, save, opts, minTrackLength);
  SERIALIZE_OPTION(section, save, opts, noiseSigmaScale);
  SERIALIZE_OPTION(section, save, opts, minStraightness);
  return true;
}

