/*
 * upscale_remap.cc
 *
 *  Created on: Nov 27, 2026
 *      Author: amyznikov
 */
#include "upscale_remap.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>

namespace {

struct alignas(16) XTap {
  int x0;
  float wx0;
  float wx1;
  float wx2;
};

} // namespace

bool upscaleRemap(cv::InputArray _srcmap, cv::OutputArray _dstmap, const cv::Size & dstSize)
{
  INSTRUMENT_REGION("");

  const cv::Size srcSize = _srcmap.size();
  if( dstSize.width <= srcSize.width || dstSize.height <= srcSize.height ) {
    CF_ERROR("Invalid output size %dx%d requested. Only upscale supported", dstSize.width, dstSize.height);
    return false;
  }

  if( _srcmap.type() != CV_32FC2 ) {
    CF_ERROR("Bad input map type: %d. Must be CV_32FC2", _srcmap.type());
    return false;
  }

  if( _dstmap.fixedType() && _dstmap.type() != CV_32FC2 ) {
    CF_ERROR("Bad output map fixed type: %d. Must be CV_32FC2", _dstmap.type());
    return false;
  }

  const cv::Mat2f src = _srcmap.getMat();
  cv::Mat2f dst = createOutOfPlace(_srcmap, _dstmap, dstSize, CV_32FC2);

  // Precompute weights and the boundaries of the safe central zone [x_begin, x_end) on X
  std::vector<XTap> xtabs(dstSize.width);
  int x_begin = 0;
  int x_end = dstSize.width;

  if ( true )  {
    const int max_x = srcSize.width - 1;
    const float scale_x = float(srcSize.width) / float(dstSize.width);

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

  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  const XTap * const __restrict xtabs_ptr = xtabs.data();

  parallel_for(0, dstSize.height, [=](const auto & range) {

    const int xbeg = x_begin;
    const int xend = x_end;
    const int max_x = srcSize.width - 1;
    const int max_y = srcSize.height - 1;
    const float scale_y = float(srcSize.height) / float(dstSize.height);

    for ( int y = rbegin(range); y < rend(range); ++y ) {
      cv::Vec2f * __restrict dstp = (cv::Vec2f *)(dst_base + y * dst_stride);

      const float fy = (float(y) + 0.5f) * scale_y - 0.5f;
      const int yc = cvFloor(fy + 0.5f);
      const float dy = fy - yc; // [-0.5, 0.5]

      const int y0 = std::max(0, std::min(yc - 1, max_y));
      const int y1 = std::max(0, std::min(yc, max_y));
      const int y2 = std::max(0, std::min(yc + 1, max_y));

      const cv::Vec2f * __restrict src0 = (const cv::Vec2f *)(src_base + y0 * src_stride);
      const cv::Vec2f * __restrict src1 = (const cv::Vec2f *)(src_base + y1 * src_stride);
      const cv::Vec2f * __restrict src2 = (const cv::Vec2f *)(src_base + y2 * src_stride);

      const float wy0 = 0.5f * (0.5f - dy) * (0.5f - dy);
      const float wy1 = 0.75f - dy * dy;
      const float wy2 = 0.5f * (0.5f + dy) * (0.5f + dy);

      // Left edge with bounds checking
      for ( int x = 0; x < xbeg; ++x ) {
        const XTap & t = xtabs_ptr[x];
        const int x0 = std::max(0, std::min(t.x0, max_x));
        const int x1 = std::max(0, std::min(t.x0 + 1, max_x));
        const int x2 = std::max(0, std::min(t.x0 + 2, max_x));

        const cv::Vec2f v0 = src0[x0] * t.wx0 + src0[x1] * t.wx1 + src0[x2] * t.wx2;
        const cv::Vec2f v1 = src1[x0] * t.wx0 + src1[x1] * t.wx1 + src1[x2] * t.wx2;
        const cv::Vec2f v2 = src2[x0] * t.wx0 + src2[x1] * t.wx1 + src2[x2] * t.wx2;

        dstp[x] = v0 * wy0 + v1 * wy1 + v2 * wy2;
      }

      // Central zone with no branching, sequential memory access
      for ( int x = xbeg; x < xend; ++x ) {
        const XTap & t = xtabs_ptr[x];
        const cv::Vec2f * __restrict p0 = src0 + t.x0;
        const cv::Vec2f * __restrict p1 = src1 + t.x0;
        const cv::Vec2f * __restrict p2 = src2 + t.x0;

        const cv::Vec2f v0 = p0[0] * t.wx0 + p0[1] * t.wx1 + p0[2] * t.wx2;
        const cv::Vec2f v1 = p1[0] * t.wx0 + p1[1] * t.wx1 + p1[2] * t.wx2;
        const cv::Vec2f v2 = p2[0] * t.wx0 + p2[1] * t.wx1 + p2[2] * t.wx2;

        dstp[x] = v0 * wy0 + v1 * wy1 + v2 * wy2;
      }

      // Right edge with bounds checking
      for ( int x = xend; x < dstSize.width; ++x ) {
        const XTap & t = xtabs_ptr[x];
        const int x0 = std::max(0, std::min(t.x0, max_x));
        const int x1 = std::max(0, std::min(t.x0 + 1, max_x));
        const int x2 = std::max(0, std::min(t.x0 + 2, max_x));

        const cv::Vec2f v0 = src0[x0] * t.wx0 + src0[x1] * t.wx1 + src0[x2] * t.wx2;
        const cv::Vec2f v1 = src1[x0] * t.wx0 + src1[x1] * t.wx1 + src1[x2] * t.wx2;
        const cv::Vec2f v2 = src2[x0] * t.wx0 + src2[x1] * t.wx1 + src2[x2] * t.wx2;

        dstp[x] = v0 * wy0 + v1 * wy1 + v2 * wy2;
      }
    }
  });

  assignOutOfPlace(_dstmap, dst);
  return true;
}
