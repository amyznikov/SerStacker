/*
 * debayer_denoise.cc
 *
 *  Created on: Nov 26, 2026
 *      Author: amyznikov
 */
#include "debayer.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if defined(__GNUC__) || defined(__clang__)
#  define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined(_MSC_VER)
#  define ALWAYS_INLINE __forceinline
#else
#  define ALWAYS_INLINE inline
#endif


template<typename _Tp1, typename _Tp2>
static bool _extract_bayer_planes(cv::InputArray _src, cv::OutputArray _dst)
{
  if( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 ) {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        _src.cols(), _src.rows(), _src.channels());
    return false;
  }

  const int rows4 = _src.rows() / 2;
  const int cols4 = _src.cols() / 2;
  const int ddepth = cv::DataType<_Tp2>::depth;

  const cv::Mat bayer_image = _src.getMat();
  const uint8_t * bayer_base = (const uint8_t * )bayer_image.ptr();
  const size_t bayer_stride = bayer_image.step;

  _dst.create(rows4, cols4, CV_MAKETYPE(ddepth, 4));
  cv::Mat & planes = _dst.getMatRef();
  uint8_t * planes_base = (uint8_t * )planes.ptr();
  const size_t planes_stride = planes.step;

  parallel_for(0, rows4, [=](const auto & range) {

    for( int y = rbegin(range); y < rend(range); ++y ) {

      const _Tp1 * src0 = (const _Tp1 *)(bayer_base + (2 * y + 0) * bayer_stride);
      const _Tp1 * src1 = (const _Tp1 *)(bayer_base + (2 * y + 1) * bayer_stride);
      _Tp2 * __restrict dstp = (_Tp2 * )(planes_base + y * planes_stride);

      for( int x = 0; x < cols4; ++x, src0 += 2, src1 += 2, dstp += 4 ) {
        dstp[0] = cv::saturate_cast<_Tp2>(src0[0]);
        dstp[1] = cv::saturate_cast<_Tp2>(src0[1]);
        dstp[2] = cv::saturate_cast<_Tp2>(src1[0]);
        dstp[3] = cv::saturate_cast<_Tp2>(src1[1]);
      }
    }
  });

  return true;
}

/** @brief
 * Extract src into 4-channel dst matrix with 4 bayer planes ordered the same as src bayer pattern
 * The output size of dst is twice smaller than src
 */
bool extract_bayer_planes(cv::InputArray src, cv::OutputArray dst)
{
  const int ddepth = dst.fixedType() ? dst.depth() : src.depth();
  CV_DISPATCH2(src.depth(),ddepth, _extract_bayer_planes, src, dst);

  return false;
}


/**
 * bayer_image: Must be single-channel bayer pattern image, or 4-channel bayer planes image
 */
template<class _Tp>
static bool _debayer_denoise(cv::Mat & _bayer_image, double _k, COLORID color_id,
    bool returnBayerPlanes)
{
  const int cn = _bayer_image.channels();
  switch (cn) {
    case 1: // assume monochrome bayer patter
      if( (_bayer_image.cols & 0x1) || (_bayer_image.rows & 0x1) ) {
        CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
            _bayer_image.cols, _bayer_image.rows, _bayer_image.channels());
        return false;
      }
      break;

    case 4: // assume already bayer planes
      break;

    default:
      CF_ERROR("Invalid number of bayer_image channels = %d (image size is %dx%dx%d)",
          _bayer_image.channels(), _bayer_image.cols, _bayer_image.rows);
      return false;
  }

  cv::Mat_<cv::Vec<_Tp, 4>> planes;
  int rows4, cols4;

  if ( cn == 4 ) {
    rows4 = _bayer_image.rows;
    cols4 = _bayer_image.cols;
    planes = _bayer_image;
  }
  else {
    rows4 = _bayer_image.rows / 2;
    cols4 = _bayer_image.cols / 2;
    _extract_bayer_planes<_Tp, _Tp>(_bayer_image, planes);
  }

  cv::Mat_<cv::Vec<_Tp, 4>> out_planes(rows4, cols4);

  const float minvar = _bayer_image.depth() < CV_32F ? 1.f : 1.f / 256.f;
  const float k = (float)_k;

  const uint8_t * const planes_base = (const uint8_t*)planes.ptr();
  const size_t planes_stride = planes.step;
  uint8_t * const out_base = (uint8_t*)out_planes.ptr();
  const size_t out_stride = out_planes.step;

  parallel_for(0, rows4, [=](const auto & range) {

    for( int y = rbegin(range); y < rend(range); ++y ) {

      const int y_prev = std::max(0, y - 1);
      const int y_next = std::min(rows4 - 1, y + 1);

      const cv::Vec<_Tp, 4> * __restrict row_prev = (const cv::Vec<_Tp, 4>*)(planes_base + y_prev * planes_stride);
      const cv::Vec<_Tp, 4> * __restrict row_curr = (const cv::Vec<_Tp, 4>*)(planes_base + y * planes_stride);
      const cv::Vec<_Tp, 4> * __restrict row_next = (const cv::Vec<_Tp, 4>*)(planes_base + y_next * planes_stride);
      cv::Vec<_Tp, 4> * __restrict dstp = (cv::Vec<_Tp, 4>*)(out_base + y * out_stride);

      cv::Vec<_Tp, 4> dst_p;

      for( int x = 0; x < cols4; ++x ) {
        const int x_prev = (x > 0) ? -1 : 0;
        const int x_next = (x < cols4 - 1) ? 1 : 0;

        const cv::Vec4f p = row_curr[x];
        const cv::Vec4f p_top = row_prev[x];
        const cv::Vec4f p_lft = row_curr[x + x_prev];
        const cv::Vec4f p_rgt = row_curr[x + x_next];
        const cv::Vec4f p_bot = row_next[x];

        #pragma unroll
        for (int c = 0; c < 4; ++c) {
          float r0 = p_top[c];
          float r1 = p_lft[c];
          float r2 = p_rgt[c];
          float r3 = p_bot[c];

          // Batcher network for median of 4 neighbors
          float t;
          if (r0 > r1) { t = r0; r0 = r1; r1 = t; }
          if (r2 > r3) { t = r2; r2 = r3; r3 = t; }
          if (r0 > r2) { t = r0; r0 = r2; r2 = t; }
          if (r1 > r3) { t = r1; r1 = r3; r3 = t; }
          if (r1 > r2) { t = r1; r1 = r2; r2 = t; }
          const float m = (r1 + r2) * 0.5f;

          // Fast local MAD
          const float mad = (std::abs((float)p_top[c] - m) +
              std::abs((float)p_lft[c] - m) +
              std::abs((float)p_rgt[c] - m) +
              std::abs((float)p_bot[c] - m)) * 0.25f;

          // Hot pixel check
          const float src_val = (float)p[c];
          dst_p[c] = (std::abs(m - src_val) > k * mad + minvar) ? (_Tp)m : (_Tp)src_val;
        }

        dstp[x] = dst_p;
      }
    }
  });

  if ( cn == 4 || returnBayerPlanes ) {
    _bayer_image = std::move(out_planes);
  }
  else {
    uint8_t * const bayer_base = (uint8_t*)_bayer_image.ptr();
    const size_t bayer_stride = _bayer_image.step;
    const uint8_t * const out_planes_base = (const uint8_t*)out_planes.ptr();
    const size_t out_planes_stride = out_planes.step;

    parallel_for(0, rows4, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        const _Tp * __restrict plane = (const _Tp *) (out_planes_base + y * out_planes_stride);
        _Tp * __restrict bayer0 = (_Tp * )(bayer_base + (2 * y + 0) * bayer_stride);
        _Tp * __restrict bayer1 = (_Tp * )(bayer_base + (2 * y + 1) * bayer_stride);
        for( int x = 0; x < cols4; ++x, plane += 4 ) {
          bayer0[2 * x + 0] = plane[0];
          bayer0[2 * x + 1] = plane[1];
          bayer1[2 * x + 0] = plane[2];
          bayer1[2 * x + 1] = plane[3];
        }
      }
    });
  }

  return true;
}


/**
 * bayer_image: Must be single-channel bayer pattern image, or 4-channel bayer planes image
 */
bool bayer_denoise(cv::Mat & bayer_image, double variation_threshold,
    COLORID color_id, bool returnBayerPlanes)
{
  if( is_bayer_pattern(color_id) ) {
    CV_DISPATCH(bayer_image.depth(), _debayer_denoise, bayer_image, variation_threshold, color_id,
        returnBayerPlanes);
  }

  CF_ERROR("Not a valid bayer pattern color_id=%d (%s) or not supported pixel depth=%d",
      color_id, toCString(color_id), bayer_image.depth());

  return false;
}
