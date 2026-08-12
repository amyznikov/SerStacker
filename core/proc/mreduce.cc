/*
 * mreduce.cc
 *
 *  Created on: Aug 12, 2026
 *      Author: amyznikov
 */

#include <core/proc/mreduce.h>
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/debug.h>
#include <limits>

template<typename _Tp1, typename _Tp2>
static bool _mreduce(cv::InputArray _src, cv::OutputArray _dst, int dim, cv::ReduceTypes rtype, int dtype,
    cv::InputArray _mask)
{
  using _MaxType = std::common_type_t<_Tp1, _Tp2>;

  constexpr int REDUCE_ROWS = 0; // 0 means that matrix is reduced to a single row
  constexpr int REDUCE_COLS = 1; // 1 means that the matrix is reduced to a single column

  const int src_rows = _src.rows();
  const int src_cols = _src.cols();
  const int channels = _src.channels();

  const int dst_rows = (dim == REDUCE_ROWS) ? 1 : src_rows;
  const int dst_cols = (dim == REDUCE_COLS) ? 1 : src_cols;

  const cv::Mat_<_Tp1> src = _src.getMat();
  const cv::Mat1b m = _mask.getMat();

  cv::Mat dst(dst_rows, dst_cols, CV_MAKETYPE(cv::DataType<_Tp2>::depth, channels));

  const uint8_t * const src_base = (uint8_t*)src.ptr();
  const size_t src_stride = src.step;

  const uint8_t * const mask_base = (uint8_t*) (m.empty() ? nullptr : m.ptr());
  const size_t mask_stride = m.step;

  uint8_t * const dst_base = (uint8_t*)dst.ptr();
  const size_t dst_stride = dst.step;

  if (dim == REDUCE_ROWS) {
    // matrix is reduced to a single row

    switch(rtype)
    {
      case cv::REDUCE_SUM: { //!< the output is the sum
        parallel_for(0, src_cols, [=](const auto & range) {

          using STp = std::conditional_t<std::is_floating_point_v<_MaxType>, _MaxType, float>;

          const int x0 = rbegin(range);

          const uint8_t * mask_xbase = mask_base ? (const uint8_t * )(mask_base) + x0 : nullptr;
          const _Tp1 * src_xbase = (const _Tp1 *) (src_base) + x0 * channels;
          _Tp2 * __restrict dstp = (_Tp2*)(dst_base) + x0 * channels;

          for ( int x = x0; x < rend(range); ++x, ++mask_xbase, src_xbase += channels, dstp += channels ) {

            STp s[channels] = {0};

            if ( !mask_base ) {
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, srcbp += src_stride ) {
                const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                for ( int c = 0; c < channels; ++c ) {
                  s[c] += srcp[c];
                }
              }
            }
            else {
              const uint8_t * mp = (const uint8_t *) (mask_xbase);
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, mp += mask_stride, srcbp += src_stride ) {
                if ( *mp ) {
                  const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] += srcp[c];
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });

        break;
      }

      case cv::REDUCE_SUM2: { //!< the output is the sum of squares
        parallel_for(0, src_cols, [=](const auto & range) {

          using STp = std::conditional_t<std::is_floating_point_v<_MaxType>, _MaxType, float>;

          const int x0 = rbegin(range);

          const uint8_t * mask_xbase = mask_base ? (const uint8_t * )(mask_base) + x0 : nullptr;
          const _Tp1 * src_xbase = (const _Tp1 *) (src_base) + x0 * channels;
          _Tp2 * __restrict dstp = (_Tp2*)(dst_base) + x0 * channels;

          for ( int x = x0; x < rend(range); ++x, ++mask_xbase, src_xbase += channels, dstp += channels ) {

            STp s[channels] = {0};

            if ( !mask_base ) {
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, srcbp += src_stride ) {
                const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                for ( int c = 0; c < channels; ++c ) {
                  const STp v = srcp[c];
                  s[c] += v * v;
                }
              }
            }
            else {
              const uint8_t * mp = (const uint8_t *) (mask_xbase);
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, mp += mask_stride, srcbp += src_stride ) {
                if ( *mp ) {
                  const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                  for ( int c = 0; c < channels; ++c ) {
                    const STp v = srcp[c];
                    s[c] += v * v;
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });

        break;
      }
      case cv::REDUCE_AVG: { //!< the output is the mean vector of all rows/columns of the matrix.
        parallel_for(0, src_cols, [=](const auto & range) {

          using STp = std::conditional_t<std::is_floating_point_v<_MaxType>, _MaxType, float>;

          const int x0 = rbegin(range);

          const uint8_t * mask_xbase = mask_base ? (const uint8_t * )(mask_base) + x0 : nullptr;
          const _Tp1 * src_xbase = (const _Tp1 *) (src_base) + x0 * channels;
          _Tp2 * __restrict dstp = (_Tp2*)(dst_base) + x0 * channels;

          for ( int x = x0; x < rend(range); ++x, ++mask_xbase, src_xbase += channels, dstp += channels ) {

            STp s[channels] = {0};
            int counts[channels] = {0};

            if ( !mask_base ) {
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, srcbp += src_stride ) {
                const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                for ( int c = 0; c < channels; ++c ) {
                  s[c] += srcp[c];
                  counts[c] += 1;
                }
              }
            }
            else {
              const uint8_t * mp = (const uint8_t *) (mask_xbase);
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, mp += mask_stride, srcbp += src_stride ) {
                if ( *mp ) {
                  const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] += srcp[c];
                    counts[c] += 1;
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(counts[c] > 1 ? s[c] / counts[c] : s[c]);
            }
          }
        });

        break;
      }

      case cv::REDUCE_MAX: { //!< the output is the maximum value
        parallel_for(0, src_cols, [=](const auto & range) {

          using STp = _Tp1;

          STp s[channels];

          const int x0 = rbegin(range);

          const uint8_t * mask_xbase = mask_base ? (const uint8_t * )(mask_base) + x0 : nullptr;
          const _Tp1 * src_xbase = (const _Tp1 *) (src_base) + x0 * channels;
          _Tp2 * __restrict dstp = (_Tp2*)(dst_base) + x0 * channels;

          for ( int x = x0; x < rend(range); ++x, ++mask_xbase, src_xbase += channels, dstp += channels ) {

            for ( int c = 0; c < channels; ++c ) {
              s[c] = std::numeric_limits<_Tp1>::lowest();
            }

            if ( !mask_base ) {
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, srcbp += src_stride ) {
                const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                for ( int c = 0; c < channels; ++c ) {
                  s[c] = std::max(s[c], srcp[c]);
                }
              }
            }
            else {
              const uint8_t * mp = (const uint8_t *) (mask_xbase);
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, mp += mask_stride, srcbp += src_stride ) {
                if ( *mp ) {
                  const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] = std::max(s[c], srcp[c]);
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });

        break;
      }
      case cv::REDUCE_MIN:  {//!< the output is the minimum (column/row-wise) of all rows/columns of the matrix.
        parallel_for(0, src_cols, [=](const auto & range) {

          using STp = _Tp1;

          STp s[channels];

          const int x0 = rbegin(range);

          const uint8_t * mask_xbase = mask_base ? (const uint8_t * )(mask_base) + x0 : nullptr;
          const _Tp1 * src_xbase = (const _Tp1 *) (src_base) + x0 * channels;
          _Tp2 * __restrict dstp = (_Tp2*)(dst_base) + x0 * channels;

          for ( int x = x0; x < rend(range); ++x, ++mask_xbase, src_xbase += channels, dstp += channels ) {

            for ( int c = 0; c < channels; ++c ) {
              s[c] = std::numeric_limits<_Tp1>::max();
            }

            if ( !mask_base ) {
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, srcbp += src_stride ) {
                const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                for ( int c = 0; c < channels; ++c ) {
                  s[c] = std::min(s[c], srcp[c]);
                }
              }
            }
            else {
              const uint8_t * mp = (const uint8_t *) (mask_xbase);
              const uint8_t * srcbp = (uint8_t*)src_xbase;
              for ( int y = 0; y < src_rows; ++y, mp += mask_stride, srcbp += src_stride ) {
                if ( *mp ) {
                  const _Tp1 * srcp = (const _Tp1 *) (srcbp);
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] = std::min(s[c], srcp[c]);
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });

        break;
      }
      default:
        CF_ERROR("Bad reduce operation %d requested", rtype);
        return false;
    }

  }
  else { // REDUCE_COLS
    // matrix is reduced to a single column

    switch(rtype)
    {
      case cv::REDUCE_SUM: { //!< the output is the sum
        parallel_for(0, src_rows, [=](const auto & range) {

          using STp = std::conditional_t<std::is_floating_point_v<_MaxType>, _MaxType, float>;

          for ( int y = rbegin(range); y < rend(range); ++y ) {
            const uint8_t * __restrict mp = mask_base ? (const uint8_t *) (mask_base + y * mask_stride) : nullptr;
            const _Tp1 * __restrict srcp = (_Tp1 *) (src_base + y * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            STp s[channels] = {0};

            if ( !mp ) {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                for ( int c = 0; c < channels; ++c ) {
                  s[c] += srcp[x * channels + c];
                }
              }
            }
            else {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                if ( *mp ) {
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] += srcp[x * channels + c];
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });

        break;
      }
      case cv::REDUCE_SUM2: { //!< the output is the sum of squares
        parallel_for(0, src_rows, [=](const auto & range) {

          using STp = std::conditional_t<std::is_floating_point_v<_MaxType>, _MaxType, float>;

          for ( int y = rbegin(range); y < rend(range); ++y ) {
            const uint8_t * __restrict mp = mask_base ? (const uint8_t *) (mask_base + y * mask_stride) : nullptr;
            const _Tp1 * __restrict srcp = (_Tp1 *) (src_base + y * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            STp s[channels] = {0};

            if ( !mp ) {
              for ( int x = 0; x < src_cols; ++x) {
                for ( int c = 0; c < channels; ++c ) {
                  const STp v = srcp[x * channels + c];
                  s[c] += v * v;
                }
              }
            }
            else {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                if ( *mp ) {
                  for ( int c = 0; c < channels; ++c ) {
                    const STp v = srcp[x * channels + c];
                    s[c] += v * v;
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });
        break;
      }
      case cv::REDUCE_AVG: { //!< the output is the average
        parallel_for(0, src_rows, [=](const auto & range) {

          using STp = std::conditional_t<std::is_floating_point_v<_MaxType>, _MaxType, float>;

          for ( int y = rbegin(range); y < rend(range); ++y ) {
            const uint8_t * __restrict mp = mask_base ? (const uint8_t *) (mask_base + y * mask_stride) : nullptr;
            const _Tp1 * __restrict srcp = (_Tp1 *) (src_base + y * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            STp s[channels] = {0};
            int counts[channels] = {0};

            if ( !mp ) {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                for ( int c = 0; c < channels; ++c ) {
                  s[c] += srcp[x * channels + c];
                  counts[c] += 1;
                }
              }
            }
            else {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                if ( *mp ) {
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] += srcp[x * channels + c];
                    counts[c] += 1;
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(counts[c] > 1 ? s[c] / counts[c] : s[c]);
            }
          }
        });

        break;
      }

      case cv::REDUCE_MAX: { //!< the output is the maximum value
        parallel_for(0, src_rows, [=](const auto & range) {

          using STp = _Tp1;
          STp s[channels];

          for ( int y = rbegin(range); y < rend(range); ++y ) {
            const uint8_t * __restrict mp = mask_base ? (const uint8_t *) (mask_base + y * mask_stride) : nullptr;
            const _Tp1 * __restrict srcp = (_Tp1 *) (src_base + y * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for ( int c = 0; c < channels; ++c ) {
              s[c] = std::numeric_limits<_Tp1>::lowest();
            }

            if ( !mp ) {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                for ( int c = 0; c < channels; ++c ) {
                  s[c] = std::max(s[c], srcp[x * channels + c]);
                }
              }
            }
            else {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                if ( *mp ) {
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] = std::max(s[c], srcp[x * channels + c]);
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });

        break;
      }
      case cv::REDUCE_MIN:  {//!< the output is the minimum value
        parallel_for(0, src_rows, [=](const auto & range) {

          using STp = _Tp1;
          STp s[channels];

          for ( int y = rbegin(range); y < rend(range); ++y ) {
            const uint8_t * __restrict mp = mask_base ? (const uint8_t *) (mask_base + y * mask_stride) : nullptr;
            const _Tp1 * __restrict srcp = (_Tp1 *) (src_base + y * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for ( int c = 0; c < channels; ++c ) {
              s[c] = std::numeric_limits<_Tp1>::max();
            }

            if ( !mp ) {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                for ( int c = 0; c < channels; ++c ) {
                  s[c] = std::min(s[c], srcp[x * channels + c]);
                }
              }
            }
            else {
              for ( int x = 0; x < src_cols; ++x, ++mp ) {
                if ( *mp ) {
                  for ( int c = 0; c < channels; ++c ) {
                    s[c] = std::min(s[c], srcp[x * channels + c]);
                  }
                }
              }
            }

            for ( int c = 0; c < channels; ++c ) {
              dstp[c] = cv::saturate_cast<_Tp2>(s[c]);
            }
          }
        });

        break;
      }
      default:
        CF_ERROR("Bad reduce operation %d requested", rtype);
        return false;
    }
  }

  _dst.move(dst);

  return true;
}

bool mreduce(cv::InputArray src, cv::OutputArray dst, int dim, cv::ReduceTypes rtype, int ddepth,
    cv::InputArray mask)
{
  if( dst.fixedType() ) {
    ddepth = dst.depth();
  }
  else if( ddepth < 0 ) {
    switch (rtype) {
      case cv::REDUCE_SUM:
        case cv::REDUCE_SUM2:
        ddepth = std::max(src.depth(), CV_32F);
        break;
      default:
        ddepth = src.depth();
        break;
    }
  }

  CV_DISPATCH2(src.depth(), ddepth, _mreduce, src, dst, dim, rtype, ddepth, mask);
  return false;
}
