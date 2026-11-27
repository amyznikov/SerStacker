/*
 * debayer_sp.cc
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


/** @brief
 * Bayer Demosaicing by 2x2 super-pixel pixel binning
 * Output dst image size is twice smaller than input src image size
 */
template<typename _Tp1, typename _Tp2>
static bool _debayer_sp(cv::InputArray _src, cv::OutputArray _dst, COLORID colorid)
{
  using Vec4T = cv::Vec<_Tp1, 4>;
  using Vec3T = cv::Vec<_Tp2, 3>;

  constexpr int c1 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 1 : 0;

  const int src_channels = _src.channels();

  if ( src_channels == 1 ) {

    const int h = _src.rows() / 2;
    const int w = _src.cols() / 2;

    const cv::Mat src = _src.getMat();
    const uint8_t * const src_base = (const uint8_t*) src.data;
    const size_t src_stride = src.step;

    _dst.create(h, w, CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
    cv::Mat & dst = _dst.getMatRef();
    uint8_t * const dst_base = (uint8_t*) dst.data;
    const size_t dst_stride = dst.step;

    switch (colorid) {
      case COLORID_BAYER_MYYC:
      case COLORID_BAYER_RGGB:
        // RGGB -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s1[1]);
              dstp[1] = _Tp2((s0[1] + s1[0] + c1) / 2);
              dstp[2] = _Tp2(s0[0]);
            }
          }
        });
        break;

      case COLORID_BAYER_YMCY:
      case COLORID_BAYER_GRBG:
        // GRBG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s1[0]);
              dstp[1] = _Tp2((c1 + s0[0] + s1[1]) / 2);
              dstp[2] = _Tp2(s0[1]);
            }
          }
        });
        break;

      case COLORID_BAYER_YCMY:
      case COLORID_BAYER_GBRG:
        // GBRG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s0[1]);
              dstp[1] = _Tp2((c1 + s0[0] + s1[1]) / 2);
              dstp[2] = _Tp2(s1[0]);
            }
          }
        });
        break;

      case COLORID_BAYER_CYYM:
      case COLORID_BAYER_BGGR:
        // BGGR -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s0[0]);
              dstp[1] = _Tp2((c1 + s0[1] + s1[0]) / 2);
              dstp[2] = _Tp2(s1[1]);
            }
          }
        });
        break;

      default: // Not supported
        CF_ERROR("Not supported colorid = %d", colorid);
        return false;
    }
  }
  else if (src_channels == 4 ) {

    const int h = _src.rows();
    const int w = _src.cols();

    const cv::Mat src = _src.getMat();
    const uint8_t * const src_base = (const uint8_t*) src.data;
    const size_t src_stride = src.step;

    _dst.create(h, w, CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
    cv::Mat & dst = _dst.getMatRef();
    uint8_t * const dst_base = (uint8_t*) dst.data;
    const size_t dst_stride = dst.step;

    switch (colorid) {
      case COLORID_BAYER_MYYC:
      case COLORID_BAYER_RGGB:
        // RGGB -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[3]); // B
              dp[1] = _Tp2((c1 + sp[1] + sp[2]) / 2);// G
              dp[2] = _Tp2(sp[0]);// R
            }
          }
        });
        break;

      case COLORID_BAYER_YMCY:
      case COLORID_BAYER_GRBG:
        // GRBG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[2]); // B
              dp[1] = _Tp2((c1 + sp[0] + sp[3]) / 2);// G
              dp[2] = _Tp2(sp[1]);// R
            }
          }
        });
        break;

      case COLORID_BAYER_YCMY:
      case COLORID_BAYER_GBRG:
        // GBRG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[1]); // B
              dp[1] = _Tp2((c1 + sp[0] + sp[3]) / 2);// G
              dp[2] = _Tp2(sp[0]);// R
            }
          }
        });
        break;

      case COLORID_BAYER_CYYM:
      case COLORID_BAYER_BGGR:
        // BGGR -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[0]); // B
              dp[1] = _Tp2((c1 + sp[1] + sp[2]) / 2);// G
              dp[2] = _Tp2(sp[3]);// R
            }
          }
        });
        break;

      default: // Not supported
        CF_ERROR("Not supported colorid = %d", colorid);
        return false;
    }
  }
  else {
    CF_ERROR("Bad number of src channels=%d", _src.channels());
    return false;
  }

  return true;
}


/** @brief
 * Bayer Demosaicing by 2x2 super-pixel pixel binning
 * Output dst image size is twice smaller than input src image size
 */
bool debayer_sp(cv::InputArray src, cv::OutputArray dst, COLORID colorid, int ddepth)
{
  if( dst.fixedType() && dst.channels() != 3 ) {
    CF_ERROR("Invalid argument: 3-channel output destination image expected but dst.channels=%d",
        dst.channels());
    return false;
  }

  if ( dst.fixedType() ) {
    ddepth = dst.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  if ( src.channels() == 1 ) {
    if( (src.cols() & 0x1) || (src.rows() & 0x1) ) {
      CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
          src.cols(), src.rows(), src.channels());
      return false;
    }
  }
  else if ( src.channels() != 4 ) {
    CF_ERROR("Not supported number of channels=%d in input image of size %dx%d",
        src.channels(), src.cols(), src.rows());
    return false;
  }


  CV_DISPATCH2(src.depth(), ddepth, _debayer_sp, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);

  return false;
}
