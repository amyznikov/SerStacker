/*
 * minmax.cc
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */
#include "minmax.h"
//#include <tbb/tbb.h>
#include <core/debug.h>

template<class T>
static bool getminmax_(cv::InputArray _src, double * minval, double * maxval, cv::InputArray _mask)
{
  // FIXME: implement with tbb

  const cv::Mat src = _src.getMat();
  const int cn = src.channels();

  T minv = static_cast<T>(DBL_MAX);
  T maxv = static_cast<T>(-DBL_MAX);

  if( _mask.empty() ) {

    if( cn == 1 ) {

      for( int y = 0; y < src.rows; ++y ) {

        const T *srcp = src.ptr<const T>(y);

        for( int x = 0; x < src.cols; ++x ) {

          const T &v = srcp[x];
          if( v > maxv ) {
            maxv = v;
          }
          if( v < minv ) {
            minv = v;
          }
        }
      }

    }
    else {

      for( int y = 0; y < src.rows; ++y ) {

        const T *srcp = src.ptr<const T>(y);

        for( int x = 0; x < src.cols; ++x ) {

          for( int c = 0; c < cn; ++c ) {

            const T &v = srcp[x * cn + c];
            if( v > maxv ) {
              maxv = v;
            }
            if( v < minv ) {
              minv = v;
            }
          }
        }
      }

    }

  }
  else if( _mask.channels() == 1 ) {

    const cv::Mat1b mask = _mask.getMat();

    if( cn == 1 ) {

      for( int y = 0; y < src.rows; ++y ) {

        const T *srcp = src.ptr<const T>(y);
        const uint8_t *mskp = mask.ptr<const uint8_t>(y);

        for( int x = 0; x < src.cols; ++x ) {
          if( mskp[x] ) {

            const T &v = srcp[x];
            if( v > maxv ) {
              maxv = v;
            }
            if( v < minv ) {
              minv = v;
            }
          }
        }
      }

    }
    else {

      for( int y = 0; y < src.rows; ++y ) {

        const T *srcp = src.ptr<const T>(y);
        const uint8_t *mskp = mask.ptr<const uint8_t>(y);

        for( int x = 0; x < src.cols; ++x ) {
          if( mskp[x] ) {

            for( int c = 0; c < cn; ++c ) {

              const T &v = srcp[x * cn + c];
              if( v > maxv ) {
                maxv = v;
              }
              if( v < minv ) {
                minv = v;
              }
            }
          }
        }
      }

    }

  }
  else if( _mask.channels() == cn ) {

    const cv::Mat mask = _mask.getMat();

    for( int y = 0; y < src.rows; ++y ) {

      const T *srcp = src.ptr<const T>(y);
      const uint8_t *mskp = mask.ptr<const uint8_t>(y);

      for( int x = 0; x < src.cols; ++x ) {

        for( int c = 0; c < cn; ++c ) {
          if( mskp[x * cn + c] ) {

            const T &v = srcp[x * cn + c];
            if( v > maxv ) {
              maxv = v;
            }
            if( v < minv ) {
              minv = v;
            }
          }
        }
      }
    }
  }
  else {
    CF_ERROR("Invalid number of channels in mask: %d. image.channels=%d", _mask.channels(), cn);
    return false;
  }

  *minval = minv;
  *maxval = maxv;

  return true;
}


bool minmax(cv::InputArray src, double * minval, double * maxval, cv::InputArray mask)
{
  switch ( src.depth() ) {

  case CV_8U :  return getminmax_<uint8_t>(src, minval, maxval, mask);
  case CV_8S :  return getminmax_<int8_t>(src, minval, maxval, mask);
  case CV_16U : return getminmax_<uint16_t>(src, minval, maxval, mask);
  case CV_16S : return getminmax_<int16_t>(src, minval, maxval, mask);
  case CV_32S : return getminmax_<int32_t>(src, minval, maxval, mask);
  case CV_32F : return getminmax_<float>(src, minval, maxval, mask);
  case CV_64F : return getminmax_<double>(src, minval, maxval, mask);
  default :
    CF_FATAL("Invalid argument: Unsupported src image type : %d", src.type());
    break;
  }

  return false;
}


