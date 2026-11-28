/*
 * weighted_average.cc
 *
 *  Created on: Nov 28, 2026
 *      Author: amyznikov
 */
#include "weighted_average.h"
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/proc/reduce_channels.h>
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




template<class _Tp, class _Tpw, bool _PerChannelWeights>
static bool _weighted_average_core(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  const cv::Size size = _src_image.size();
  const int cn = _src_image.channels();

  constexpr bool has_weights = !std::is_void_v<_Tpw>;
  const int wcn = has_weights ? _src_weights.channels() : 0;

  if constexpr ( has_weights ) {
    if ( wcn != 1 && (!_PerChannelWeights || wcn != cn) ) {
      CF_ERROR("Invalid weights channels=%d for image channels=%d and accw channels=%d",
          wcn, cn, _weights_accumulator.channels());
      return false;
    }
  }

  const cv::Mat src = _src_image.getMat();
  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  const cv::Mat srcw = has_weights ? _src_weights.getMat() : cv::Mat();
  const uint8_t * const srcw_base = has_weights ? srcw.ptr() : nullptr;
  const size_t srcw_stride = has_weights ? srcw.step : 0;

  cv::Mat & acc = _src_accumulator.getMatRef();
  uint8_t * const acc_base = acc.ptr();
  const size_t acc_stride = acc.step;

  cv::Mat & W = _weights_accumulator.getMatRef();
  uint8_t * const accw_base = W.ptr();
  const size_t accw_stride = W.step;

  parallel_for(0, size.height, [=](const auto & range) {
    const int y0 = rbegin(range);
    const int y1 = rend(range);
    for ( int y = y0; y < y1; ++y ) {
      const _Tp * srcp = (const _Tp *)(src_base + y * src_stride);
      float * __restrict accp = (float *)(acc_base + y * acc_stride);
      float * __restrict accwp = (float *)(accw_base + y * accw_stride);
      const _Tpw * wp = has_weights ? (const _Tpw *)(srcw_base + y * srcw_stride) : nullptr;

      for ( int x = 0; x < size.width; ++x, srcp += cn, accp += cn ) {

        if constexpr ( !_PerChannelWeights ) {
          // 1-channel weight accumulator (CV_32FC1)
          float w_new = 1.0f;
          if constexpr ( has_weights ) {
            if constexpr ( std::is_integral_v<_Tpw> ) {
              w_new = (*wp++) ? 1.0f : 0.0f;
            }
            else {
              w_new = *wp++;
            }
          }

          if ( w_new > 0.0f ) {
            const float W_new = *accwp + w_new;
            const float factor = w_new / W_new;
            *accwp = W_new;

            for ( int c = 0; c < cn; ++c ) {
              const float I_new = srcp[c];
              const float A_old = accp[c];
              accp[c] = A_old + (I_new - A_old) * factor;
            }
          }
          ++accwp;
        }
        else {
          // Per-channel weight accumulator (CV_32FC(cn))
          if constexpr ( !has_weights ) {
            for ( int c = 0; c < cn; ++c ) {
              const float W_new = accwp[c] + 1.0f;
              const float factor = 1.0f / W_new;
              accwp[c] = W_new;

              const float I_new = srcp[c];
              const float A_old = accp[c];
              accp[c] = A_old + (I_new - A_old) * factor;
            }
          }
          else if ( wcn == 1 ) {
            // Single-channel weights applied to per-channel accumulator
            float w_new;
            if constexpr ( std::is_integral_v<_Tpw> ) {
              w_new = (*wp++) ? 1.0f : 0.0f;
            }
            else {
              w_new = *wp++;
            }

            if ( w_new > 0.0f ) {
              for ( int c = 0; c < cn; ++c ) {
                const float W_new = accwp[c] + w_new;
                const float factor = w_new / W_new;
                accwp[c] = W_new;

                const float I_new = float(srcp[c]);
                const float A_old = accp[c];
                accp[c] = A_old + (I_new - A_old) * factor;
              }
            }
          }
          else {
            // Multi-channel weights (wcn == cn)
            for ( int c = 0; c < cn; ++c ) {
              float w_new;
              if constexpr ( std::is_integral_v<_Tpw> ) {
                w_new = wp[c] ? 1.0f : 0.0f;
              }
              else {
                w_new = wp[c];
              }

              if ( w_new > 0.0f ) {
                const float W_new = accwp[c] + w_new;
                const float factor = w_new / W_new;
                accwp[c] = W_new;

                const float I_new = srcp[c];
                const float A_old = accp[c];
                accp[c] = A_old + (I_new - A_old) * factor;
              }
            }
            wp += cn;
          }
          accwp += cn;
        }
      }
    }
  });

  return true;
}

template<class _Tp, class _Tpw>
static bool _weighted_average_dispatch_wcn(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  const int accw_cn = _weights_accumulator.channels();
  if ( accw_cn == 1 ) {
    return _weighted_average_core<_Tp, _Tpw, false>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
  }
  if ( accw_cn == _src_image.channels() ) {
    return _weighted_average_core<_Tp, _Tpw, true>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
  }

  CF_ERROR("Invalid weights_accumulator channels=%d (expected 1 or %d)", accw_cn, _src_image.channels());
  return false;
}

template<class _Tp>
static bool _weighted_average_update(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  if ( _src_weights.empty() ) {
    return _weighted_average_dispatch_wcn<_Tp, void>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
  }

  switch ( _src_weights.depth() ) {
    case CV_8U:
      return _weighted_average_dispatch_wcn<_Tp, uint8_t>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    case CV_16U:
      return _weighted_average_dispatch_wcn<_Tp, uint16_t>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    case CV_32F:
      return _weighted_average_dispatch_wcn<_Tp, float>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    case CV_64F:
      return _weighted_average_dispatch_wcn<_Tp, double>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    default:
      break;
  }

  CF_ERROR("Unsupported _src_weights depth=%d in weighted_average_update", _src_weights.depth());
  return false;
}

bool weighted_average_update(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  INSTRUMENT_REGION("");

  const cv::Size size = _src_image.size();
  const int cn = _src_image.channels();

  if ( _src_accumulator.size() != size || _weights_accumulator.size() != size ) {
    CF_ERROR("Image and accumulator sizes not match");
    return false;
  }

  if ( _src_accumulator.depth() != CV_32F || _weights_accumulator.depth() != CV_32F ) {
    CF_ERROR("Bad accumulator depth: acc=%d, accw=%d (must be CV_32F)",
        _src_accumulator.depth(), _weights_accumulator.depth());
    return false;
  }

  if ( _src_accumulator.channels() != cn ) {
    CF_ERROR("Bad number of image channels=%d. Must be %d", cn, _src_accumulator.channels());
    return false;
  }

  if ( !_src_weights.empty() && _src_weights.size() != size ) {
    CF_ERROR("Image size (%dx%d) and weights size (%dx%d) not match",
        size.width, size.height, _src_weights.cols(), _src_weights.rows());
    return false;
  }

  CV_DISPATCH(_src_image.depth(), _weighted_average_update, _src_image, _src_weights,
      _src_accumulator, _weights_accumulator);

  CF_ERROR("APP BUG: BAD _src_image.depth()=%d encountered", _src_image.depth());
  return false;
}
