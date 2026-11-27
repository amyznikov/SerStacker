/*
 * debayer_nn.cc
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




template<class _Tp1, class _Tp2>
bool _debayer_nn2_interpolation(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  if ( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 )  {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        _src.cols(), _src.rows(), _src.channels());
    return false;
  }

  constexpr int c1 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 1 : 0;
  constexpr int c2 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 2 : 0;

  const cv::Size size = _src.size();

 const cv::Mat src = _src.getMat();

  _dst.create(size, CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
  cv::Mat & dst = _dst.getMatRef();

  #define CAPTURE_PARAMS \
    bayer_base = src.data, \
    dst_base = dst.data, \
    src_stride = src.step, \
    dst_stride = dst.step, \
    size, c1, c2

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1   = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // R G
            dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4); // B
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4); // G
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]); // R
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2); // B
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]); // G
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2); // R
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4); // B
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4); // G
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]); // R
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2); // B
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]); // G
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2); // R
            }
          }
          else { // G B
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2); // B
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]); // G
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2); // R
            dstp[3] = cv::saturate_cast<_Tp2>(s1[1]); // B
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4); // G
            dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4); // R
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2); // B
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]); // G
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2); // R
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]); // B
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4); // G
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4); // R
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1 = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // G R
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2);
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2);
            dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4);
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4);
            dstp[5] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }
          }
          else { // B G
            dstp[0] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1 = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // G B
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2);
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2);
            dstp[3] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4);
            dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
            }
          }
          else { // R G
            dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2);
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1 = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // B G
            dstp[0] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }
          }
          else { // G R
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2);
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2);
            dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4);
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4);
            dstp[5] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }
          }
        }
      });
      break;
    }

    default:
      CF_ERROR("Not supported colorid=%d (%s) for debayer",(int)(colorid), toCString(colorid));
      return false;
  }

  #undef CAPTURE_PARAMS

//  _dst.move(dst);
  return true;
}



bool debayer_nn2(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, int ddepth)
{
  if ( !is_bayer_pattern(colorid) ) {
    CF_ERROR("colorid=%d (%s) is not bayer pattern", (int)colorid, toCString(colorid));
    return false;
  }

  if ( src.channels() != 1 ) {
    CF_ERROR("Input image channels=%d. Must be 1", src.channels());
    return false;
  }

  if( dst.fixedType() ) {
    ddepth = dst.depth();
    if (dst.channels() != 3 ) {
      CF_ERROR("Output image channels=%d. Must be 3", dst.channels());
      return false;
    }
  }
  else if( ddepth < 0 ) {
    ddepth = src.depth();
  }

  CV_DISPATCH2(src.depth(), ddepth, _debayer_nn2_interpolation, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);

  return false;
}
