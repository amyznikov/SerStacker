/*
 * divide.cc
 *
 *  Created on: Sep 1, 2026
 *      Author: amyznikov
 */

#include <core/proc/divide.h>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>

template<class _Tp1, class _Tp2, class _Tp3>
static bool _divideImages(cv::InputArray _src1, cv::InputArray _src2, cv::OutputArray _dst, double eps)
{
  using _Tpc = std::common_type_t<std::common_type_t<_Tp1,_Tp2>, _Tp3>;
  using _Tcomp = std::conditional_t<std::is_floating_point_v<_Tpc>, _Tpc, float>;

  const cv::Size size = _src1.size();

  const cv::Mat src1 = _src1.getMat();
  const uint8_t * const src1_base = (uint8_t*) src1.ptr();
  const size_t src1_stride = src1.step;
  const int src1_channels = _src1.channels();

  const cv::Mat src2 = _src2.getMat();
  const uint8_t * const src2_base = (uint8_t*) src2.ptr();
  const size_t src2_stride = src2.step;
  const int src2_channels = _src2.channels();

  _dst.create(size, CV_MAKETYPE(cv::DataType<_Tp3>::depth, src1_channels));
  cv::Mat & dst = _dst.getMatRef();
  uint8_t * const dst_base = (uint8_t*) dst.ptr();
  const size_t dst_stride = dst.step;

  const _Tcomp abs_eps = static_cast<_Tcomp>(std::abs(eps));

  parallel_for(0, size.height, [=](const auto & range) {

    for ( int y = rbegin(range); y < rend(range); ++y) {
      const _Tp1* srcp1 = (const _Tp1* )(src1_base + y * src1_stride);
      const _Tp2* srcp2 = (const _Tp2* )(src2_base + y * src2_stride);
      _Tp3* __restrict dstp = (_Tp3* )(dst_base + y * dst_stride);

      if ( src1_channels == 1 && src2_channels == 1 ) {
        for ( int x = 0; x < size.width; ++x, ++srcp1, ++srcp2, ++dstp) {
          const _Tcomp v = *srcp2;
          *dstp = cv::saturate_cast<_Tp3>( std::abs(v) > abs_eps ? *srcp1 / v : 0);
        }
      }
      else if ( src2_channels == 1 ) {
        for ( int x = 0; x < size.width; ++x, srcp1 += src1_channels, ++srcp2, dstp += src1_channels ) {
          const _Tcomp v = *srcp2;
          if ( std::abs(v) > abs_eps ) {
            const _Tcomp inv_v = 1 / v;
            for ( int c = 0; c < src1_channels; ++c) {
              dstp[c] = cv::saturate_cast<_Tp3>(srcp1[c] * inv_v);
            }
          }
          else {
            for ( int c = 0; c < src1_channels; ++c) {
              dstp[c] = 0;
            }
          }
        }
      }
      else {
        for ( int x = 0, total_pixels = src1_channels * size.width; x < total_pixels; ++x, ++srcp1, ++srcp2, ++dstp ) {
          const _Tcomp v = *srcp2;
          *dstp = cv::saturate_cast<_Tp3>( std::abs(v) > abs_eps ? *srcp1 / v : 0);
        }
      }
    }
  });

  return true;
}

bool divideImages(cv::InputArray src1, cv::InputArray src2, cv::OutputArray dst, double eps, int ddepth)
{
  if ( src1.empty() || src2.empty() ) {
    dst.release();
    return true;
  }

  if ( src1.size() != src2.size() ) {
    CF_ERROR("Size mismatch: src1.size()=%dx%d, src2.size()=%dx%d",
        src1.cols(), src1.rows(), src2.cols(), src2.rows());
    return false;
  }

  if ( src2.channels() != 1 && src2.channels() != src1.channels() ) {
    CF_ERROR("Not supported channels combination: src1.channels=%d, src2.channels=%d",
        src1.channels(), src2.channels());
    return false;
  }

  if( dst.fixedType() ) {
    ddepth = dst.depth();
  }
  else if( ddepth < 0 ) {
    ddepth = src1.depth();
  }

  const bool success =
       cv_dispatch_helper(src1.depth(), [&](auto t1) {
         using T1 = std::remove_pointer_t<decltype(t1)>;
         return cv_dispatch_helper(src2.depth(), [&](auto t2) {
           using T2 = std::remove_pointer_t<decltype(t2)>;
           return cv_dispatch_helper(ddepth, [&](auto t3) {
             using T3 = std::remove_pointer_t<decltype(t3)>;
             return _divideImages<T1, T2, T3>(src1, src2, dst, eps);
           });
         });
       });

  if (!success) {
    CF_ERROR("Unsupported pixel depth combination encountered: src1=%d, src2=%d, dst=%d",
        src1.depth(), src2.depth(), ddepth);
    return false;
  }

  return true;
}


