/*
 * c_frame_accumulation.cc
 *
 *  Created on: Feb 14, 2021
 *      Author: amyznikov
 */

#include "c_frame_accumulation.h"
#include <core/proc/fft.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/unsharp_mask.h>
#include <tbb/tbb.h>
#include <core/debug.h>


template<class T1, class T2>
static bool divide_accumulator_(const cv::Mat & acc, const cv::Mat & weights,
    cv::OutputArray rdata, cv::OutputArray rmask,
    double scale, int ddepth)
{

  const int cn = acc.channels();

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
  else if ( rdata.needed() ) {

    cv::Mat & D = rdata.getMatRef();

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
  else if ( rmask.needed() ) {
    cv::compare(weights, 0, rmask, cv::CMP_GT);
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
    cv::Mat & acc, cv::Mat & accw, int accdepth)
{

  if ( acc.empty() || accw.empty() ) {
    acc.create(src.size(), CV_MAKETYPE(accdepth, src.channels()));
    accw.create(src.size(), acc.depth());
    acc.setTo(0);
    accw.setTo(0);
  }

  const int cn = acc.channels();

  typedef tbb::blocked_range<int> range;
  const int grain_size = 512;

  const cv::Mat S = src.getMat();
  const cv::Mat W = weights.getMat();

  tbb::parallel_for(range(0, S.rows, grain_size),
        [&S, &W, &acc, &accw, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

            const T1 * sp = S.ptr<const T1>(y);
            const T2 * wp = W.ptr<const T2>(y);

            T3 * accp = acc.ptr<T3>(y);
            T3 * accwp = accw.ptr<T3>(y);

            for ( int x = 0, n = acc.cols; x < n; ++x ) {
              const double w = wp[x];
              if ( w > 0 ) {
                accwp[x] += w;
                for ( int c = 0; c < cn; ++c ) {
                  accp[x * cn + c] += sp[x * cn + c] * w;
                }
              }
            }
          }
  });


  return true;
}

static bool accumulate_weighted(cv::InputArray src, cv::InputArray weights,
    cv::Mat & acc, cv::Mat & accw, int accdepth)
{
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

bool c_frame_weigthed_average::add(cv::InputArray src, cv::InputArray mask)
{
  if ( accumulator_.empty() || counter_.empty() ) {
    accumulator_.create(src.size(), CV_MAKETYPE(accdepth, src.channels()));
    counter_.create(src.size(), accumulator_.depth());
    accumulator_.setTo(0);
    counter_.setTo(0);
    accumulated_frames_ = 0;
  }
  else if ( src.size() != accumulator_.size() ) {
    CF_ERROR("ERROR in weigthed_frame_average: current frame and accumulator sizes not match");
    return false;
  }
  else if ( src.channels() != accumulator_.channels() ) {
    CF_ERROR("ERROR in weigthed_frame_average: current frame and accumulator channel count not match");
    return false;
  }

  if ( !mask.empty() ) {

    if ( mask.size() != src.size() ) {
      CF_ERROR("ERROR in weigthed_frame_average: iimage and mask sizes not match");
      return false;
    }

    if ( mask.channels() != 1 ) {
      CF_ERROR("ERROR in weigthed_frame_average: mask must be 1-channel binary or floating point matrix");
      return false;
    }
  }


  if ( mask.empty() || mask.depth() == CV_8U ) {
    cv::add(accumulator_, src, accumulator_, mask, accumulator_.type());
    cv::add(counter_, 1, counter_, mask, counter_.type());
  }
  else if ( !accumulate_weighted(src, mask, accumulator_, counter_, accdepth) ) {
    CF_ERROR("ERROR in weigthed_frame_average: accumulate_weighted() fails");
    return false;
  }

  ++accumulated_frames_;
  return true;

}

bool c_frame_weigthed_average::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  if ( !divide_accumulator(accumulator_, counter_, avg, mask, dscale, ddepth) ) {
    CF_ERROR("ERROR in weigthed_frame_average: divide_accumulator() fails");
    return false;
  }

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

//bool c_frame_accumulation_with_mask::add(cv::InputArray src, cv::InputArray mask)
//{
//  if ( accumulator_.empty() || counter_.empty() ) {
//    accumulator_.create(src.size(), CV_MAKETYPE(accdepth, src.channels()));
//    counter_.create(src.size(), accumulator_.depth());
//    accumulator_.setTo(0);
//    counter_.setTo(0);
//  }
//  else if ( src.size() != accumulator_.size() ) {
//    CF_ERROR("ERROR: current frame and accumulator sizes not match");
//    return false;
//  }
//
//  cv::add(accumulator_, src, accumulator_, mask, accumulator_.type());
//  cv::add(counter_, 1, counter_, mask, counter_.type());
//
//  ++accumulated_frames_;
//
//  return true;
//}
//
//bool c_frame_accumulation_with_mask::compute(cv::OutputArray avg, cv::OutputArray mask, double scale, int ddepth) const
//{
//  if ( accumulator_.empty() || counter_.empty() ) {
//    CF_ERROR("c_frame_accumulation_with_mask: accumulator is empty");
//    return false;
//  }
//
//  if ( !divide_accumulator(accumulator_, counter_, avg, mask, scale, ddepth) ) {
//    CF_ERROR("c_frame_accumulation_with_mask: divide_accumulator() fails");
//    return false;
//  }
//
//  return true;
//}
//
//cv::Size c_frame_accumulation_with_mask::accumulator_size() const
//{
//  return accumulator_.size();
//}
//
//const cv::Mat & c_frame_accumulation_with_mask::accumulator() const
//{
//  return accumulator_;
//}
//
//const cv::Mat & c_frame_accumulation_with_mask::counter() const
//{
//  return counter_;
//}
//
//void c_frame_accumulation_with_mask::release()
//{
//
//  accumulator_.release();
//  counter_.release();
//  accumulated_frames_ = 0;
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//bool c_frame_accumulation_with_weights::add(cv::InputArray src, cv::InputArray weights)
//{
//
//  if ( accumulate_weighted(src, weights, accumulator_, weights_, accdepth) ) {
//    ++accumulated_frames_;
//    return true;
//  }
//  return false;
//}
//
//bool c_frame_accumulation_with_weights::compute(cv::OutputArray avg, cv::OutputArray mask, double scale, int ddepth) const
//{
//  if ( accumulator_.empty() || weights_.empty() ) {
//    CF_ERROR("c_frame_accumulation_with_weights: accumulator is empty");
//    return false;
//  }
//
//  if ( !divide_accumulator(accumulator_, weights_, avg, mask, scale, ddepth) ) {
//    CF_ERROR("c_frame_accumulation_with_weights: divide_accumulator() fails");
//    return false;
//  }
//  return true;
//}
//
//void c_frame_accumulation_with_weights::release()
//{
//  accumulator_.release();
//  weights_.release();
//  tmp_.release();
//  accumulated_frames_ = 0;
//}
//
//cv::Size c_frame_accumulation_with_weights::accumulator_size() const
//{
//  return accumulator_.size();
//}
//
//const cv::Mat & c_frame_accumulation_with_weights::accumulator() const
//{
//  return accumulator_;
//}
//
//const cv::Mat & c_frame_accumulation_with_weights::weights() const
//{
//  return weights_;
//}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  const cv::Size src_size = channels[0].size();

  if ( accumulators_.empty() ) {

    fftSize_ = getOptimalFFTSize(src_size, cv::Size(0,0), false);

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


void c_sharpness_norm_measure::set_norm_type(cv::NormTypes v)
{
  norm_type_ = v;
}

cv::NormTypes c_sharpness_norm_measure::norm_type() const
{
  return norm_type_;
}


double c_sharpness_norm_measure::sigma() const
{
  return sigma_;
}

void c_sharpness_norm_measure::set_sigma(double v)
{
  sigma_ = v;
}

double c_sharpness_norm_measure::measure(cv::InputArray src, cv::InputArray mask, double sigma, cv::NormTypes norm_type)
{
  return hpass_norm(src, sigma, mask, norm_type);
}

double c_sharpness_norm_measure::measure(cv::InputArray src, cv::InputArray mask) const
{
  return measure(src, mask, sigma_, norm_type_);
}

double  c_sharpness_norm_measure::add(cv::InputArray src, cv::InputArray mask)
{
  const double v = measure(src, mask);
  accumulator_ += v;
  counter_ += 1;
  return v;
}

double c_sharpness_norm_measure::average() const
{
  return accumulator_ / counter_;
}

void c_sharpness_norm_measure::reset()
{
  accumulator_ = 0;
  counter_ = 0;
}


double c_sharpness_norm_measure::accumulator() const
{
  return accumulator_;
}

int c_sharpness_norm_measure::counter() const
{
  return counter_;
}

