/*
 * c_frame_accumulation.cc
 *
 *  Created on: Feb 14, 2021
 *      Author: amyznikov
 */

#include "c_frame_accumulation.h"
#include <tbb/tbb.h>
#include <core/proc/fft.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/laplacian_pyramid.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<class T1, class T2>
static bool divide_accumulator_(const cv::Mat & acc, const cv::Mat & weights,
    cv::OutputArray rdata, cv::OutputArray rmask,
    double scale, int ddepth)
{

  const int cn =
      acc.channels();

  if ( rdata.needed() ) {
    rdata.create(acc.size(), CV_MAKETYPE(ddepth, cn));
  }

  if ( rmask.needed() ) {
    rmask.create(acc.size(), CV_8UC1);
  }

  typedef tbb::blocked_range<int> range;
  const int grain_size = 512;

  if ( rdata.needed() && rmask.needed() ) {

    cv::Mat & D = rdata.getMatRef();
    cv::Mat & M = rmask.getMatRef();

    if( acc.channels() == 1 && weights.channels() == 1 ) {

      tbb::parallel_for(range(0, acc.rows, grain_size),
          [&acc, &weights, &D, &M, cn](const range & r) {
            for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

              const T1 * accp =
                  acc.ptr<const T1>(y);

              const T1 * wp =
                  weights.ptr<const T1>(y);

              T2 * resp =
                  D.ptr<T2>(y);

              uint8_t * mskp =
                  M.ptr<uint8_t>(y);

              for ( int x = 0, n = acc.cols; x < n; ++x ) {
                if ( wp[x] > 0 ) {
                  resp[x] = cv::saturate_cast<T2> (accp[x] / wp[x]);
                  mskp[x] = 255;
                }
                else {
                  mskp[x] = 0;
                  resp[x] = 0;
                }
              }
            }
          });

    }

    else if ( acc.channels() == weights.channels() ) {

      tbb::parallel_for(range(0, acc.rows, grain_size),
          [&acc, &weights, &D, &M, cn](const range & r) {
            for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

              const T1 * accp =
                  acc.ptr<const T1>(y);

              const T1 * wp =
                  weights.ptr<const T1>(y);

              T2 * resp =
                  D.ptr<T2>(y);

              uint8_t * mskp =
                  M.ptr<uint8_t>(y);

              for ( int x = 0, n = acc.cols; x < n; ++x ) {
                mskp[x] = 255;
                for ( int c = 0; c < cn; ++c ) {
                  if ( wp[x * cn + c] > 0 ) {
                    resp[x * cn + c ] = cv::saturate_cast<T2> (accp[x * cn + c ] / wp[x * cn + c]);
                  }
                  else {
                    mskp[x] = 0;
                    resp[x * cn + c ] = 0;
                  }
                }
              }
            }
          });

    }
    else if (weights.channels() == 1) {

      double wmin, wmax;
      cv::minMaxLoc(weights, &wmin, &wmax);
      CF_DEBUG("weights: min=%g max=%g", wmin, wmax);

      tbb::parallel_for(range(0, acc.rows, grain_size),
          [&acc, &weights, &D, &M, cn](const range & r ) {
            for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

              const T1 * accp = acc.ptr<const T1>(y);
              const T1 * weightsp = weights.ptr<const T1>(y);

              T2 * resp = D.ptr<T2>(y);
              uint8_t * mskp = M.ptr<uint8_t>(y);

              for ( int x = 0, n = acc.cols; x < n; ++x ) {
                const double w = weightsp[x];
                if ( w > 0 ) {
                  mskp[x] = 255;
                  for ( int c = 0; c < cn; ++c ) {
                    resp[x * cn + c ] = cv::saturate_cast<T2> (accp[x * cn + c ] / w );
                  }
                }
                else {
                  mskp[x] = 0;
                  for ( int c = 0; c < cn; ++c ) {
                    resp[x * cn + c ] = 0;
                  }
                }
              }
            }
          });

    }
    else {
      CF_ERROR("Unsupported combination of acc (%d) and weights (%d) channels",
          acc.channels(), weights.channels());
      return false;
    }

  }
  else if ( rdata.needed() ) {

    cv::Mat & D = rdata.getMatRef();

    if ( acc.channels() == weights.channels() ) {

      tbb::parallel_for(range(0, acc.rows, grain_size),
          [&acc, &weights, &D, cn](const range & r) {
            for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

              const T1 * accp =
                  acc.ptr<const T1>(y);

              const T1 * wp =
                  weights.ptr<const T1>(y);

              T2 * resp =
                  D.ptr<T2>(y);

              for ( int x = 0, n = acc.cols; x < n; ++x ) {
                for ( int c = 0; c < cn; ++c ) {
                  if ( wp[x * cn + c] > 0 ) {
                    resp[x * cn + c ] = cv::saturate_cast<T2> (accp[x * cn + c ] / wp[x * cn + c]);
                  }
                  else {
                    resp[x * cn + c ] = 0;
                  }
                }
              }
            }
          });

    }
    else if (weights.channels() == 1) {

      double wmin, wmax;
      cv::minMaxLoc(weights, &wmin, &wmax);
      CF_DEBUG("weights: min=%g max=%g", wmin, wmax);

      tbb::parallel_for(range(0, acc.rows, grain_size),
          [&acc, &weights, &D, cn](const range & r ) {
            for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

              const T1 * accp = acc.ptr<const T1>(y);
              const T1 * weightsp = weights.ptr<const T1>(y);

              T2 * resp = D.ptr<T2>(y);

              for ( int x = 0, n = acc.cols; x < n; ++x ) {
                const double w = weightsp[x];
                if ( w > 0 ) {
                  for ( int c = 0; c < cn; ++c ) {
                    resp[x * cn + c ] = cv::saturate_cast<T2> (accp[x * cn + c ] / w );
                  }
                }
                else {
                  for ( int c = 0; c < cn; ++c ) {
                    resp[x * cn + c ] = 0;
                  }
                }
              }
            }
          });

    }
    else {
      CF_ERROR("Unsupported combination of acc (%d) and weights (%d) channels",
          acc.channels(), weights.channels());
      return false;
    }

  }
  else if ( rmask.needed() ) {
    if ( weights.channels() == 1 ) {
      cv::compare(weights, 0, rmask, cv::CMP_GT);
    }
    else {
      cv::Mat tmp;
      reduce_color_channels(weights, tmp, cv::REDUCE_MIN);
      cv::compare(tmp, 0, rmask, cv::CMP_GT);
    }
  }

  return true;
}

static bool divide_accumulator(const cv::Mat & acc, const cv::Mat & weights,
    cv::OutputArray rdata, cv::OutputArray rmask,
    double scale, int ddepth)
{
  if ( rdata.needed() && rdata.fixedType()) {
    ddepth = rdata.depth();
  }

  if ( ddepth < 0 || ddepth > CV_64F ) {
    ddepth = acc.depth();
  }

  switch ( acc.depth() ) {
  case CV_32F :
    switch ( ddepth ) {
    case CV_32F :
      return divide_accumulator_<float, float>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_64F :
      return divide_accumulator_<float, double>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8U :
      return divide_accumulator_<float, uint8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8S :
      return divide_accumulator_<float, int8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16U :
      return divide_accumulator_<float, uint16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16S :
      return divide_accumulator_<float, int16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_32S :
      return divide_accumulator_<float, int32_t>(acc, weights, rdata, rmask, scale, ddepth);
    }
    break;
  case CV_64F:
    switch ( ddepth ) {
    case CV_32F :
      return divide_accumulator_<double, float>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_64F :
      return divide_accumulator_<double, double>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8U :
      return divide_accumulator_<double, uint8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8S :
      return divide_accumulator_<double, int8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16U :
      return divide_accumulator_<double, uint16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16S :
      return divide_accumulator_<double, int16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_32S :
      return divide_accumulator_<double, int32_t>(acc, weights, rdata, rmask, scale, ddepth);
    }
    break;
  case CV_8U :
    switch ( ddepth ) {
    case CV_32F :
      return divide_accumulator_<uint8_t, float>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_64F :
      return divide_accumulator_<uint8_t, double>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8U :
      return divide_accumulator_<uint8_t, uint8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8S :
      return divide_accumulator_<uint8_t, int8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16U :
      return divide_accumulator_<uint8_t, uint16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16S :
      return divide_accumulator_<uint8_t, int16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_32S :
      return divide_accumulator_<uint8_t, int32_t>(acc, weights, rdata, rmask, scale, ddepth);
    }
    break;
  case CV_8S :
    switch ( ddepth ) {
    case CV_32F :
      return divide_accumulator_<int8_t, float>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_64F :
      return divide_accumulator_<int8_t, double>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8U :
      return divide_accumulator_<int8_t, uint8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8S :
      return divide_accumulator_<int8_t, int8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16U :
      return divide_accumulator_<int8_t, uint16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16S :
      return divide_accumulator_<int8_t, int16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_32S :
      return divide_accumulator_<int8_t, int32_t>(acc, weights, rdata, rmask, scale, ddepth);
    }
    break;
  case CV_16U:
    switch ( ddepth ) {
    case CV_32F :
      return divide_accumulator_<uint16_t, float>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_64F :
      return divide_accumulator_<uint16_t, double>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8U :
      return divide_accumulator_<uint16_t, uint8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8S :
      return divide_accumulator_<uint16_t, int8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16U :
      return divide_accumulator_<uint16_t, uint16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16S :
      divide_accumulator_<uint16_t, int16_t>(acc, weights, rdata, rmask, scale, ddepth);
      break;
    case CV_32S :
      return divide_accumulator_<uint16_t, int32_t>(acc, weights, rdata, rmask, scale, ddepth);
    }
    break;
  case CV_16S:
    switch ( ddepth ) {
    case CV_32F :
      return divide_accumulator_<int16_t, float>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_64F :
      return divide_accumulator_<int16_t, double>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8U :
      return divide_accumulator_<int16_t, uint8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8S :
      return divide_accumulator_<int16_t, int8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16U :
      return divide_accumulator_<int16_t, uint16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16S :
      return divide_accumulator_<int16_t, int16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_32S :
      return divide_accumulator_<int16_t, int32_t>(acc, weights, rdata, rmask, scale, ddepth);
    }
    break;
  case CV_32S:
    switch ( ddepth ) {
    case CV_32F :
      return divide_accumulator_<int32_t, float>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_64F :
      return divide_accumulator_<int32_t, double>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8U :
      return divide_accumulator_<int32_t, uint8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_8S :
      return divide_accumulator_<int32_t, int8_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16U :
      return divide_accumulator_<int32_t, uint16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_16S :
      return divide_accumulator_<int32_t, int16_t>(acc, weights, rdata, rmask, scale, ddepth);
    case CV_32S :
      return divide_accumulator_<int32_t, int32_t>(acc, weights, rdata, rmask, scale, ddepth);
    }
    break;
  }

  return false;
}


template<class T1, class T2, class T3>
static bool accumulate_weighted_(cv::InputArray src, cv::InputArray weights,
    cv::Mat & acc, cv::Mat & counter, int accdepth)
{
  if( acc.empty() || counter.empty() ) {
    acc.create(src.size(), CV_MAKETYPE(accdepth, src.channels()));
    counter.create(src.size(), CV_MAKETYPE(accdepth, weights.channels()));
    acc.setTo(0);
    counter.setTo(0);
  }

  const int cn = acc.channels();

  if( weights.channels() != counter.channels() ) {
    CF_ERROR("Number of channels in weight mask was changed: "
        "counter.channels=%d weights.channels=%d",
        counter.channels(), weights.channels());
    return false;
  }


  typedef tbb::blocked_range<int> range;
  const int grain_size = 512;

  const cv::Mat S = src.getMat();
  const cv::Mat W = weights.getMat();

  if( src.channels() == 1 && weights.channels() == 1 ) {

    tbb::parallel_for(range(0, S.rows, grain_size),
        [&S, &W, &acc, &counter](const range & r) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

            const T1 * sp = S.ptr<const T1>(y);
            const T2 * wp = W.ptr<const T2>(y);

            T3 * accp = acc.ptr<T3>(y);
            T3 * cntp = counter.ptr<T3>(y);

            for ( int x = 0, n = acc.cols; x < n; ++x ) {
              cntp[x] += wp[x];
              accp[x] += sp[x] * wp[x];
            }
          }
        });

  }
  else if( src.channels() == weights.channels() ) {

    tbb::parallel_for(range(0, S.rows, grain_size),
        [&S, &W, &acc, &counter, cn](const range & r) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

            const T1 * sp = S.ptr<const T1>(y);
            const T2 * wp = W.ptr<const T2>(y);

            T3 * accp = acc.ptr<T3>(y);
            T3 * cntp = counter.ptr<T3>(y);

            for ( int x = 0, n = acc.cols; x < n; ++x ) {
              for ( int c = 0; c < cn; ++c ) {
                cntp[x * cn + c] += wp[x * cn + c];
                accp[x * cn + c] += sp[x * cn + c] * wp[x * cn + c];
              }
            }
          }
        });
  }
  else if( weights.channels() == 1 ) {

    tbb::parallel_for(range(0, S.rows, grain_size),
        [&S, &W, &acc, &counter, cn](const range & r) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

            const T1 * sp = S.ptr<const T1>(y);
            const T2 * wp = W.ptr<const T2>(y);

            T3 * accp = acc.ptr<T3>(y);
            T3 * cntp = counter.ptr<T3>(y);

            for ( int x = 0, n = acc.cols; x < n; ++x ) {
              cntp[x] += wp[x];
              for ( int c = 0; c < cn; ++c ) {
                accp[x * cn + c] += sp[x * cn + c] * wp[x];
              }
            }
          }
        });
  }
  else {
    CF_ERROR("Unsupported combination of image (%d) and weights (%d) channels",
        src.channels(), weights.channels());
    return false;
  }


  return true;
}

static bool accumulate_weighted(cv::InputArray src, cv::InputArray weights,
    cv::Mat & acc, cv::Mat & accw)
{
  CF_DEBUG("weights.empty()=%d weights.type()=%d", weights.empty(), weights.type());

  const int accdepth = acc.depth();

  switch ( src.depth() ) {
  case CV_8U :
    switch ( weights.depth() ) {
    case CV_32F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint8_t, float, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint8_t, float, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_64F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint8_t, double, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint8_t, double, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint8_t, uint8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint8_t, uint8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint8_t, int8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint8_t, int8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint8_t, uint16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint8_t, uint16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint8_t, int16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint8_t, int16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_32S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint8_t, int32_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint8_t, int32_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    }
    break;

  case CV_8S :
    switch ( weights.depth() ) {
    case CV_32F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int8_t, float, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int8_t, float, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_64F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int8_t, double, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int8_t, double, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int8_t, uint8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int8_t, uint8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int8_t, int8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int8_t, int8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int8_t, uint16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int8_t, uint16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int8_t, int16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int8_t, int16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_32S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int8_t, int32_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int8_t, int32_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    }
    break;

  case CV_16U :
    switch ( weights.depth() ) {
    case CV_32F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint16_t, float, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint16_t, float, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_64F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint16_t, double, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint16_t, double, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint16_t, uint8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint16_t, uint8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint16_t, int8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint16_t, int8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint16_t, uint16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint16_t, uint16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint16_t, int16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint16_t, int16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_32S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<uint16_t, int32_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<uint16_t, int32_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    }
    break;

  case CV_16S :
    switch ( weights.depth() ) {
    case CV_32F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int16_t, float, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int16_t, float, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_64F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int16_t, double, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int16_t, double, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int16_t, uint8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int16_t, uint8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int16_t, int8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int16_t, int8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int16_t, uint16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int16_t, uint16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int16_t, int16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int16_t, int16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_32S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int16_t, int32_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int16_t, int32_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    }
    break;

  case CV_32S :
    switch ( weights.depth() ) {
    case CV_32F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int32_t, float, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int32_t, float, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_64F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int32_t, double, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int32_t, double, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int32_t, uint8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int32_t, uint8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int32_t, int8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int32_t, int8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int32_t, uint16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int32_t, uint16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int32_t, int16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int32_t, int16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_32S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<int32_t, int32_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<int32_t, int32_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    }
    break;
  case CV_32F :
    switch ( weights.depth() ) {
    case CV_32F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<float, float, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<float, float, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_64F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<float, double, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<float, double, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<float, uint8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<float, uint8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<float, int8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<float, int8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<float, uint16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<float, uint16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<float, int16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<float, int16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_32S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<float, int32_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<float, int32_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    }
    break;
  case CV_64F :
    switch ( weights.depth() ) {
    case CV_32F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<double, float, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<double, float, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_64F :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<double, double, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<double, double, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<double, uint8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<double, uint8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_8S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<double, int8_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<double, int8_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16U :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<double, uint16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<double, uint16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_16S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<double, int16_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<double, int16_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    case CV_32S :
      switch ( accdepth ) {
      case CV_32F :
        return accumulate_weighted_<double, int32_t, float>(src, weights, acc, accw, accdepth);
      case CV_64F :
        return accumulate_weighted_<double, int32_t, double>(src, weights, acc, accw, accdepth);
      }
      break;
    }
    break;
  }

  return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_frame_weigthed_average::c_frame_weigthed_average()
{

}

c_frame_weigthed_average::c_frame_weigthed_average(const cv::Size & image_size, int acctype, int weightstype)
{
  initialze(image_size, acctype, weightstype);
}

bool c_frame_weigthed_average::initialze(const cv::Size & image_size, int acctype, int weightstype)
{

  const int accdepth =
      CV_MAT_DEPTH(acctype);

  const int cntdepth =
      CV_MAT_DEPTH(weightstype);

  const int acn =
      (acctype >> CV_CN_SHIFT) + 1;

  const int ccn =
      (weightstype >> CV_CN_SHIFT) + 1;

  if( ccn != 1 && ccn != acn ) {
    CF_ERROR("Invalid ombination of acc (%d) and weights (%d) channels", acn, ccn);
    return false;
  }

  accumulator_.create(image_size, acctype);
  counter_.create(image_size, CV_MAKETYPE(std::max(CV_32F, CV_MAT_DEPTH(weightstype)), ccn));

  accumulator_.setTo(0);
  counter_.setTo(0);
  accumulated_frames_ = 0;

  return true;
}

bool c_frame_weigthed_average::add(cv::InputArray src, cv::InputArray weights)
{
  INSTRUMENT_REGION("");

  if ( src.size() != accumulator_.size() ) {
    CF_ERROR("ERROR in weigthed_frame_average: current frame (%dx%d) and accumulator (%dx%d) sizes not match",
        src.cols(), src.rows(), accumulator_.cols, accumulator_.rows );
    return false;
  }
  else if ( src.channels() != accumulator_.channels() ) {
    CF_ERROR("ERROR in weigthed_frame_average: current frame (%d) and accumulator (%d) channel count not match",
        src.channels(), accumulator_.channels());
    return false;
  }

  if( !weights.empty() && src.size() != weights.size() ) {
    CF_ERROR("ERROR in weigthed_frame_average: image size=%dx%d and weights size = %dx%d not match",
        src.cols(), src.rows(),
        weights.cols(), weights.rows());
    return false;
  }


  if ( weights.empty() || weights.type() == CV_8UC1 ) {

    cv::add(accumulator_, src, accumulator_, weights, accumulator_.type());
    cv::add(counter_, cv::Scalar::all(1), counter_, weights, counter_.type());
  }
  else if ( !accumulate_weighted(src, weights, accumulator_, counter_) ) {
    CF_ERROR("ERROR in weigthed_frame_average: accumulate_weighted() fails");
    return false;
  }

  ++accumulated_frames_;

  return true;
}

bool c_frame_weigthed_average::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  if ( accumulated_frames_ < 1 ) {
    CF_ERROR("No frames was accumulated");
    return false;
  }

  CF_DEBUG("accumulator_: %dx%d channels=%d depth=%d counter_: %dx%d channels=%d depth=%d",
      accumulator_.cols, accumulator_.rows,
      accumulator_.channels(), accumulator_.depth(),
      counter_.cols, counter_.rows,
      counter_.channels(), counter_.depth());


  INSTRUMENT_REGION("");

  cv::Mat m;
  cv::compare(counter_, 0, m, cv::CMP_GT);
  if( m.channels() > 1 ) {
    reduce_color_channels(m, m, cv::REDUCE_MIN);
  }

  if( avg.needed() ) {

    cv::Mat cc;

    if( counter_.channels() == accumulator_.channels() ) {
      cc = counter_;
    }
    else {
      std::vector<cv::Mat> channels(accumulator_.channels(), counter_);
      cv::merge(channels, cc);
    }

    cv::divide(accumulator_, cc, avg);
    avg.setTo(0, ~m);
  }

  if( mask.needed() ) {
    mask.move(m);
  }


//  if ( !divide_accumulator(accumulator_, counter_, avg, mask, dscale, ddepth) ) {
//    CF_ERROR("ERROR in weigthed_frame_average: divide_accumulator() fails");
//    return false;
//  }

  return true;
}

void c_frame_weigthed_average::release()
{
  accumulator_.release();
  counter_.release();
  accumulated_frames_ = 0;
}

cv::Size c_frame_weigthed_average::accumulator_size() const
{
  return accumulator_.size();
}

const cv::Mat & c_frame_weigthed_average::accumulator() const
{
  return accumulator_;
}

const cv::Mat & c_frame_weigthed_average::counter() const
{
  return counter_;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<>
const c_enum_member* members_of<c_laplacian_pyramid_focus_stacking::fusing_policy>()
{
  static constexpr c_enum_member members[] = {

      { c_laplacian_pyramid_focus_stacking::select_max_energy, "select_max_energy",
          "select max laplacian energy" },

      { c_laplacian_pyramid_focus_stacking::weighted_average, "weighted_average",
          "average with weighting by laplacian energy" },

      { c_laplacian_pyramid_focus_stacking::select_max_energy },
  };
  return members;
}


c_laplacian_pyramid_focus_stacking::c_laplacian_pyramid_focus_stacking(const options & opts) :
    opts_(opts)
{
}

cv::Mat c_laplacian_pyramid_focus_stacking::duplicate_channels(const cv::Mat & src, int cn)
{
  cv::Mat m;
  cv::merge(std::vector<cv::Mat>(cn, src), m);
  return m;
}

bool c_laplacian_pyramid_focus_stacking::initialze(const cv::Size & image_size, int acctype, int weightstype)
{
  acc.clear();
  G.release();
  image_size_ = image_size;
  acctype_ = acctype;
  weightstype_ = weightstype;
  accumulated_frames_ = 0;
  return true;
}


bool c_laplacian_pyramid_focus_stacking::add(cv::InputArray src, cv::InputArray mask)
{
  static const auto graystdev =
      [](const cv::Mat & image) -> double {
        cv::Scalar m, s;
        cv::meanStdDev(image, m, s);
        double sv = s[0];
        for ( int i = 1, cn = image.channels(); i < cn; ++i ) {
          sv += s[i];
        }
        return sv;
      };

  static const auto apply_mask =
      [](std::vector<cv::Mat> & lpyr, cv::InputArray m) {

        if( !m.empty() && m.type() == CV_8UC1 ) {

          std::vector<cv::Mat> mskpyr;

          cv::buildPyramid(m, mskpyr, lpyr.size() - 1);

          for( int i = 0, n = mskpyr.size(); i < n - 1; ++i ) {
            cv::compare(mskpyr[i], 255, mskpyr[i], cv::CMP_LT);
            if( !cv::countNonZero(mskpyr[i]) ) {
              break;
            }
            lpyr[i].setTo(0, mskpyr[i]);
          }
        }
      };

  static const auto compute_energy =
      [](cv::Mat & lap, cv::Mat & w, const cv::Mat & G, bool avgc) {

        if ( !avgc || lap.channels() == 1 ) {
          cv::multiply(lap, lap, w);
        }
        else {
          reduce_color_channels(lap, w, cv::REDUCE_SUM);
          cv::multiply(w, w, w);
        }

        if ( !G.empty() ) {
          cv::sepFilter2D(w, w, -1, G, G, cv::Point(-1, -1), 1e-12);
        }

        if ( lap.channels() == w.channels() ) {
          cv::multiply(lap, w, lap);
        }
        else {
          cv::multiply(lap, duplicate_channels(w, lap.channels()), lap);
        }
      };


  const cv::Mat image =
      src.getMat();

  if( image.size() != image_size_ ) {

    CF_ERROR("Input image size %dx%d not match: expected %dx%d",
        image.cols, image.rows,
        image_size_.width, image_size_.height);

    return false;
  }

  if( opts_.inpaint_mask_holes ) {
    linear_interpolation_inpaint(image, mask, image);
  }

  if( acc.empty() ) {

    if( G.empty() && (opts_.ksigma > 0 || opts_.kradius > 0) ) {

      G = cv::getGaussianKernel(std::max(0, 2 * opts_.kradius + 1),
          std::max(0., opts_.ksigma),
          CV_32F);
    }

    build_laplacian_pyramid(image, acc, 8);
    apply_mask(acc, mask);

    if( opts_.fusing_policy == weighted_average ) {

      wwp.resize(acc.size() - 1);
      for( int i = 0, n = acc.size(); i < n - 1; ++i ) {
        compute_energy(acc[i], wwp[i], G, opts_.avgchannel);
      }
    }

    ++accumulated_frames_;
    return true;
  }

  if( image.channels() != acc.front().channels() ) {
    CF_ERROR("Number of channels in input image %d not match to accumulator channels %d",
        image.channels(), acc.front().channels());
    return false;
  }


  std::vector<cv::Mat> pyr;
  cv::Mat w[2], ww, m;

  build_laplacian_pyramid(image, pyr, 8);
  apply_mask(pyr, mask);


  const int pyrsize =
      pyr.size();

  if( pyrsize != acc.size() ) {
    CF_ERROR("UNEXPECTED APP BUG: current pyramid size %zu not match to acc pyramid size %zu",
        pyr.size(), acc.size());
    return false;
  }

  const double sv[2] = {
      graystdev(acc.back()),
      graystdev(pyr.back())
  };

  cv::addWeighted(acc.back(), sv[0] / (sv[0] + sv[1]),
      pyr.back(), sv[1] / (sv[0] + sv[1]),
      0,
      acc.back());

  const int cn =
      image.channels();

  for( int i = 0; i < pyrsize - 1; ++i ) {

    switch (opts_.fusing_policy) {
      case select_max_energy: {

        cv::absdiff(acc[i], 0, w[0]);
        cv::absdiff(pyr[i], 0, w[1]);

        if( !G.empty() ) {
          cv::sepFilter2D(w[0], w[0], -1, G, G, cv::Point(-1, -1));
          cv::sepFilter2D(w[1], w[1], -1, G, G, cv::Point(-1, -1));
        }

        cv::compare(w[1], w[0], m, cv::CMP_GT);
        pyr[i].copyTo(acc[i], m);

        break;
      }

      case weighted_average:
        default: {
        compute_energy(pyr[i], ww, G, opts_.avgchannel);
        cv::add(pyr[i], acc[i], acc[i]);
        cv::add(ww, wwp[i], wwp[i]);
        break;
      }
    }
  }

  ++accumulated_frames_;

  return true;
}

bool c_laplacian_pyramid_focus_stacking::compute(cv::OutputArray avg, cv::OutputArray mask,
    double dscale, int ddepth) const
{
  switch (opts_.fusing_policy) {
    case select_max_energy:
      reconstruct_laplacian_pyramid(avg, acc);
      break;

    case weighted_average:
      default:
      if( !acc.empty() ) {
        std::vector<cv::Mat> lpyr(acc.size());
        cv::Mat w;
        for( int i = 0, n = acc.size(); i < n - 1; ++i ) {
          if( acc[i].channels() == wwp[i].channels() ) {
            cv::divide(acc[i], wwp[i], lpyr[i]);
          }
          else {
            cv::divide(acc[i], duplicate_channels(wwp[i], acc[i].channels()), lpyr[i]);
          }
        }
        lpyr.back() = acc.back();
        reconstruct_laplacian_pyramid(avg, lpyr);
      }
      break;
  }

  return true;
}

void c_laplacian_pyramid_focus_stacking::release()
{
  acc.clear();
  G.release();
  accumulated_frames_ = 0;
}

cv::Size c_laplacian_pyramid_focus_stacking::accumulator_size() const
{
  return image_size_;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool c_frame_accumulation_with_fft::initialze(const cv::Size & /*image_size*/, int /*acctype*/, int /*weightstype*/)
{
  accumulators_.clear();
  weights_.clear();
  return true;
}

bool c_frame_accumulation_with_fft::add(cv::InputArray src, cv::InputArray _w)
{
  (void)(_w);


  const int nc = src.channels();

  if ( !accumulators_.empty() && accumulators_.size() != nc ) {
    CF_ERROR("Number of channels not match, expected %zu channel input image", accumulators_.size());
    return  false;
  }

  cv::Mat channels[nc];
  cv::Mat weights[nc];

  if ( nc == 1 ) {
    src.getMat().copyTo(channels[0]);
  }
  else {
    cv::split(src.getMat(), channels);
  }

  const cv::Size src_size =
      channels[0].size();

  if ( accumulators_.empty() ) {

    fftSize_ = fftGetOptimalSize(src_size, cv::Size(0,0), false);

    CF_DEBUG("src_size=%dx%d fftSize_=%dx%d",
        src_size.width, src_size.height,
        fftSize_.width, fftSize_.height);


    if ( src_size == fftSize_ ) {
      rc_.x = rc_.y = rc_.width = rc_.height = 0;
    }
    else {
      border_top_ = (fftSize_.height - src_size.height) / 2;
      border_bottom_ = (fftSize_.height - src_size.height - border_top_);
      border_left_ = (fftSize_.width - src_size.width) / 2;
      border_right_ = (fftSize_.width - src_size.width - border_left_);
      rc_ = cv::Rect(border_left_, border_top_, src.cols(), src.rows());

    }

  }

  if ( nc == 1 ) {
    if ( src_size == fftSize_ ) {
      cv::dft(channels[0], channels[0],
          cv::DFT_COMPLEX_OUTPUT);
    }
    else {

      cv::copyMakeBorder(channels[0], channels[0],
          border_top_, border_bottom_,
          border_left_, border_right_,
          cv::BORDER_REFLECT);

      cv::dft(channels[0], channels[0],
          cv::DFT_COMPLEX_OUTPUT);

      fftPower(channels[0], weights[0], true);
    }
  }
  else {

    tbb::parallel_for(0, nc,
        [this, src_size, &channels, &weights](int i ) {

          if ( src_size == fftSize_ ) {
            cv::dft(channels[i], channels[i],
                cv::DFT_COMPLEX_OUTPUT);
          }
          else {

            cv::Mat tmp;

            cv::copyMakeBorder(channels[i], tmp,
                border_top_, border_bottom_,
                border_left_, border_right_,
                cv::BORDER_REFLECT);

            channels[i] = tmp;

            cv::dft(channels[i], channels[i],
                cv::DFT_COMPLEX_OUTPUT);

          }

          fftPower(channels[i], weights[i], false);
        });
  }

  if ( accumulators_.empty() ) {

    accumulators_.resize(nc);
    weights_.resize(nc);

    for ( int i = 0; i < nc; ++i ) {

      accumulators_[i].create(channels[i].size(), channels[i].type());
      accumulators_[i].setTo(0);

      weights_[i].create(weights[i].size(), weights[i].type());
      weights_[i].setTo(0);
    }

  }


  for ( int i = 0; i < nc; ++i ) {

    //double min, max;
    //cv::minMaxLoc(weights[i], &min, &max);
    //CF_DEBUG("weights      [i=%d]: countNaNs=%d min=%g max=%g", i, countNaNs(weights[i]), min, max);

    cv::accumulateProduct(channels[i], weights[i], accumulators_[i]);
    cv::accumulate(weights[i], weights_[i]);
  }


  ++accumulated_frames_;

  return true;
}

bool c_frame_accumulation_with_fft::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{

  const int nc = accumulators_.size();
  if ( nc < 1 || accumulators_[0].empty() ) {
    CF_ERROR("c_frame_accumulation_with_fft: accumulator is empty");
    return false;
  }


  cv::Mat channels[nc];

  if ( ddepth < 0 ) {
    ddepth = accumulators_[0].depth();
  }

  for ( int i = 0; i < nc; ++i ) {
    //double min, max;

    cv::divide(accumulators_[i], weights_[i], channels[i], dscale, ddepth);

    //cv::minMaxLoc(channels[i], &min, &max);
    //CF_DEBUG("channels     [i=%d]: countNaNs=%d min=%g max=%g", i, countNaNs(channels[i]), min, max);

    cv::idft(channels[i], channels[i], cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
    //cv::minMaxLoc(channels[i], &min, &max);
    //CF_DEBUG("channels     [i=%d]: countNaNs=%d min=%g max=%g", i, countNaNs(channels[i]), min, max);
  }

  if ( rc_.empty() ) {
    if ( nc == 1 ) {
      avg.move(channels[0]);
    }
    else {
      cv::merge(channels, nc, avg);
    }
  }
  else {
    if ( nc == 1 ) {
      channels[0](rc_).copyTo(avg);
    }
    else {
      cv::Mat tmp;
      cv::merge(channels, nc, tmp);
      tmp(rc_).copyTo(avg);
    }
  }

  if ( mask.needed() ) {
    cv::Mat1b m(avg.size(), 255);
    mask.move(m);
  }


  return true;
}

void c_frame_accumulation_with_fft::release()
{
  accumulated_frames_ = 0;
  accumulators_.clear();
  weights_.clear();
  rc_.x = rc_.y = rc_.width = rc_.height = 0;
  fftSize_.width =  fftSize_.height = 0;
  border_top_ = 0;
  border_bottom_ = 0;
  border_left_ = 0;
  border_right_ = 0;
}

cv::Size c_frame_accumulation_with_fft::accumulator_size() const
{
  return accumulators_.empty() ? cv::Size(0,0) : accumulators_[0].size();
}

const std::vector<cv::Mat> & c_frame_accumulation_with_fft::accumulators() const
{
  return accumulators_;
}

const std::vector<cv::Mat> & c_frame_accumulation_with_fft::weights() const
{
  return weights_;
}

int c_frame_accumulation_with_fft::countNaNs(const cv::Mat & image)
{
  int cnt = 0;

  const int nc = image.channels();

  for ( int y = 0; y < image.rows; ++y ) {

    const float * p = image.ptr<const float>(y);

    for ( int x = 0; x < image.cols * nc; ++x ) {
      if ( isnan(p[x]) ) {
        ++cnt;
      }
    }
  }

  return cnt;
}

double c_frame_accumulation_with_fft::power(double x)
{
  return x * x * x;
}

double c_frame_accumulation_with_fft::square(double x)
{
  return x * x;
}

bool c_frame_accumulation_with_fft::fftPower(const cv::Mat & src, cv::Mat & dst, bool mc)
{
  if ( src.channels() != 2 || src.depth() != CV_32F ) {
    CF_ERROR("invalid arg: FP32 2-channel input image expected");
    return false;
  }

  const cv::Mat2f csrc = src;
  cv::Mat2f cmag;

  cmag.create(src.size());

  double scale = square(1. / src.size().area());
  CF_DEBUG("scale=%g", scale);


  if ( !mc ) {
    for ( int y = 0; y < csrc.rows; ++y ) {
      for ( int x = 0; x < csrc.cols; ++x ) {
        const double a = csrc[y][x][0];
        const double b = csrc[y][x][1];
        const double p = power((a * a + b * b) * scale);
        cmag[y][x][0] = cmag[y][x][1] = std::max(scale, p);
      }
    }
  }
  else {

    tbb::parallel_for(0, csrc.rows,
        [&csrc, &cmag, scale](int y) {
          for ( int x = 0; x < csrc.cols; ++x ) {
            const double a = csrc[y][x][0];
            const double b = csrc[y][x][1];
            const double p = power((a * a + b * b) * scale);
            cmag[y][x][0] = cmag[y][x][1] = std::max(scale, p);
          }
        });
  }

  dst = std::move(cmag);

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void c_bayer_average::set_bayer_pattern(COLORID colorid)
{
  colorid_ = colorid;
  if ( !accumulator_.size().empty() ) {
    generate_bayer_pattern_mask();
  }
}

COLORID c_bayer_average::bayer_pattern() const
{
  return colorid_;
}

void c_bayer_average::set_remap(const cv::Mat2f & rmap)
{
  rmap_ = rmap;
}

const cv::Mat2f & c_bayer_average::remap() const
{
  return rmap_ ;
}

bool c_bayer_average::initialze(const cv::Size & image_size, int /*acctype*/, int /*weightstype*/)
{
  accumulator_.create(image_size);
  counter_.create(image_size);

  accumulator_.setTo(0);
  counter_.setTo(0);

  accumulated_frames_ = 0;

  generate_bayer_pattern_mask();

  return true;
}



template<class BT>
static void bayer_accumulate(cv::InputArray bayer_image, cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const cv::Mat1b & bayer_pattern,
    const cv::Mat & weigths)
{
  typedef tbb::blocked_range<int> range;
  constexpr int tbb_grain_size = 128;

  const cv::Mat_<BT> src = bayer_image.getMat();

  if( rmap.empty() ) {

    if( weigths.empty() ) {

      tbb::parallel_for(range(0, acc.rows, tbb_grain_size),
          [&](const range & r) {

            for ( int y = r.begin(); y < r.end(); ++y ) {
              for( int x = 0; x < acc.cols; ++x ) {
                // color channel for update
                const int cc = bayer_pattern[y][x];
                acc[y][x][cc] += src[y][x];
                cntr[y][x][cc] += 1;
            }
          }
        });

    }
    else if( weigths.type() == CV_8UC1 ) {

      const cv::Mat1b &w = weigths;

      tbb::parallel_for(range(0, acc.rows, tbb_grain_size),
          [&](const range & r) {

            for ( int y = r.begin(); y < r.end(); ++y ) {
              for( int x = 0; x < acc.cols; ++x ) {
                if ( w[y][x] ) {
                  // color channel for update
                const int cc = bayer_pattern[y][x];
                acc[y][x][cc] += src[y][x];
                cntr[y][x][cc] += 1;
            }
          }
        }
      });

    }
    else if( weigths.type() == CV_32FC1 ) {

      const cv::Mat1f &w = weigths;

      tbb::parallel_for(range(0, acc.rows, tbb_grain_size),
          [&](const range & r) {

            for ( int y = r.begin(); y < r.end(); ++y ) {
              for( int x = 0; x < acc.cols; ++x ) {
                // color channel for update
                const int cc = bayer_pattern[y][x];
                acc[y][x][cc] += src[y][x] * w[y][x];
                cntr[y][x][cc] += w[y][x];
            }
          }
        });

    }

  }
  else {

    static const auto interpolate =
        [](int x, int y, const cv::Vec2f & p, const cv::Mat_<BT> & src, cv::Mat3f & acc, cv::Mat3f & cntr,
            const cv::Mat1b & bayer_pattern, float w) {

              const int src_x = (int)(p[0]);
              const int src_y = (int)(p[1]);

              if( src_x >= 0 && src_x < src.cols - 1 && src_y >= 0 && src_y < src.rows - 1 ) {

                // select color channels and pixel weights for update

                const float ax = (src_x + 1 - p[0]);// occupied x side on [src_x] pixel
                const float ay = (src_y + 1 - p[1]);// occupied y side on [src_y] pixel
                const float bx = (p[0] - src_x);// occupied x side on [src_x+1] pixel
                const float by = (p[1] - src_y);// occupied y side on [src_y+1] pixel


                const float s00 = ax * ay * w;
                const int c00 = bayer_pattern[src_y + 0][src_x + 0];
                acc[y][x][c00] += src[src_y + 0][src_x + 0] * s00;
                cntr[y][x][c00] += s00;


                const float s01 = bx * ay * w;
                const int c01 = bayer_pattern[src_y + 0][src_x + 1];
                acc[y][x][c01] += src[src_y + 0][src_x + 1] * s01;
                cntr[y][x][c01] += s01;


                const float s10 = ax * by * w;
                const int c10 = bayer_pattern[src_y + 1][src_x + 0];
                acc[y][x][c10] += src[src_y + 1][src_x + 0] * s10;
                cntr[y][x][c10] += s10;


                const float s11 = bx * by * w;
                const int c11 = bayer_pattern[src_y + 1][src_x + 1];
                acc[y][x][c11] += src[src_y + 1][src_x + 1] * s11;
                cntr[y][x][c11] += s11;
              }
        };

    if( weigths.empty() ) {

      tbb::parallel_for(range(0, acc.rows, tbb_grain_size),
          [&](const range & r) {

            for ( int y = r.begin(); y < r.end(); ++y ) {

              const cv::Vec2f *rmp = rmap[y];

              for( int x = 0; x < acc.cols; ++x ) {
                interpolate(x, y, rmp[x], src, acc, cntr, bayer_pattern, 1);
              }
            }
          });
    }
    else if( weigths.type() == CV_8UC1 ) {

      const cv::Mat1b w = weigths;

      tbb::parallel_for(range(0, acc.rows, tbb_grain_size),
          [&](const range & r) {
            for ( int y = r.begin(); y < r.end(); ++y ) {
              const cv::Vec2f *rmp = rmap[y];
              for( int x = 0; x < acc.cols; ++x ) {
                if ( w[y][x] ) {
                  interpolate(x, y, rmp[x], src, acc, cntr, bayer_pattern, 1);
                }
              }
            }
          });

    }
    else if( weigths.type() == CV_32FC1 ) {

      const cv::Mat1f w = weigths;

      tbb::parallel_for(range(0, acc.rows, tbb_grain_size),
          [&](const range & r) {
            for ( int y = r.begin(); y < r.end(); ++y ) {
              const cv::Vec2f *rmp = rmap[y];
              for( int x = 0; x < acc.cols; ++x ) {
                interpolate1f(x, y, rmp[x], src, acc, cntr, bayer_pattern, w[y][x]);
              }
            }
          });

    }


  }

}

bool c_bayer_average::add(cv::InputArray src, cv::InputArray weights)
{
  const cv::Mat src_bayer =
      src.getMat();

  const cv::Mat w =
      weights.getMat();

  switch (src_bayer.type()) {
    case CV_8UC1:
      bayer_accumulate<uint8_t>(src, accumulator_, counter_, rmap_, bayer_pattern_, w);
      break;
    case CV_8SC1:
      bayer_accumulate<int8_t>(src, accumulator_, counter_, rmap_, bayer_pattern_, w);
      break;
    case CV_16UC1:
      bayer_accumulate<uint16_t>(src, accumulator_, counter_, rmap_, bayer_pattern_, w);
      break;
    case CV_16SC1:
      bayer_accumulate<int16_t>(src, accumulator_, counter_, rmap_, bayer_pattern_, w);
      break;
    case CV_32SC1:
      bayer_accumulate<int32_t>(src, accumulator_, counter_, rmap_, bayer_pattern_, w);
      break;
    case CV_32FC1:
      bayer_accumulate<float>(src, accumulator_, counter_, rmap_, bayer_pattern_, w);
      break;
    case CV_64FC1:
      bayer_accumulate<double>(src, accumulator_, counter_, rmap_, bayer_pattern_, w);
      break;
    default:
      break;
  }

  ++accumulated_frames_;

  return true;
}

bool c_bayer_average::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  if( accumulated_frames_ < 1 ) {
    return false;
  }

  cv::Mat3f img(accumulator_.size(), 0.f);

  for( int y = 0; y < img.rows; ++y ) {

    for( int x = 0; x < img.cols; ++x ) {

      for( int c = 0; c < 3; ++c ) {

        if( counter_[y][x][c] > 0 ) {
          img[y][x][c] = accumulator_[y][x][c] / counter_[y][x][c];
        }
        else {
          img[y][x][c] = 0;
        }
      }
    }
  }

  avg.move(img);

  return true;
}

void c_bayer_average::release()
{
  accumulator_.release();
  counter_.release();
  rmap_.release();
  bayer_pattern_.release();
}


cv::Size c_bayer_average::accumulator_size() const
{
  return accumulator_.size();
}

const cv::Mat & c_bayer_average::accumulator() const
{
  return accumulator_;
}

const cv::Mat & c_bayer_average::counter() const
{
  return counter_;
}

void c_bayer_average::generate_bayer_pattern_mask()
{
  bayer_pattern_.create(accumulator_.size());

  switch (colorid_) {
    case COLORID_BAYER_RGGB:
      /*
       * R G
       * G B
       * */
      for ( int y = 0; y < bayer_pattern_.rows/2; ++y ) {
        for ( int x = 0; x < bayer_pattern_.cols/2; ++x ) {
          bayer_pattern_[2 * y + 0][2 * x + 0] = BAYER_R;
          bayer_pattern_[2 * y + 0][2 * x + 1] = BAYER_G;
          bayer_pattern_[2 * y + 1][2 * x + 0] = BAYER_G;
          bayer_pattern_[2 * y + 1][2 * x + 1] = BAYER_B;
        }
      }
      break;


    case COLORID_BAYER_GRBG:
      /*
       * G R
       * B G
       * */
      for ( int y = 0; y < bayer_pattern_.rows/2; ++y ) {
        for ( int x = 0; x < bayer_pattern_.cols/2; ++x ) {
          bayer_pattern_[2 * y + 0][2 * x + 0] = BAYER_G;
          bayer_pattern_[2 * y + 0][2 * x + 1] = BAYER_R;
          bayer_pattern_[2 * y + 1][2 * x + 0] = BAYER_B;
          bayer_pattern_[2 * y + 1][2 * x + 1] = BAYER_G;
        }
      }
      break;
    case COLORID_BAYER_GBRG:
      /*
       * G B
       * R G
       * */
      for ( int y = 0; y < bayer_pattern_.rows/2; ++y ) {
        for ( int x = 0; x < bayer_pattern_.cols/2; ++x ) {
          bayer_pattern_[2 * y + 0][2 * x + 0] = BAYER_G;
          bayer_pattern_[2 * y + 0][2 * x + 1] = BAYER_B;
          bayer_pattern_[2 * y + 1][2 * x + 0] = BAYER_R;
          bayer_pattern_[2 * y + 1][2 * x + 1] = BAYER_G;
        }
      }
      break;
    case COLORID_BAYER_BGGR:
      /*
       * B G
       * G R
       * */
      for ( int y = 0; y < bayer_pattern_.rows/2; ++y ) {
        for ( int x = 0; x < bayer_pattern_.cols/2; ++x ) {
          bayer_pattern_[2 * y + 0][2 * x + 0] = BAYER_B;
          bayer_pattern_[2 * y + 0][2 * x + 1] = BAYER_G;
          bayer_pattern_[2 * y + 1][2 * x + 0] = BAYER_G;
          bayer_pattern_[2 * y + 1][2 * x + 1] = BAYER_R;
        }
      }
      break;
    default:
      break;
  }


}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
