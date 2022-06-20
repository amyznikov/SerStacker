/*
 * median.cc
 *
 *  Created on: Nov 11, 2016
 *      Author: amyznikov
 */

#include "median.h"
#include "normalize.h"
#include <core/debug.h>



#define CV_MTYPE_SWITCH_1(m, call, args) \
    switch ( (m).type() ) { \
      case CV_8UC1  : return call<uint8_t> args; break; \
      case CV_8SC1  : return call<int8_t> args; break; \
      case CV_16UC1 : return call<uint16_t> args; break; \
      case CV_16SC1 : return call<int16_t> args; break; \
      case CV_32SC1 : return call<int32_t> args; break; \
      case CV_32FC1 : return call<float> args; break; \
      case CV_64FC1 : return call<double> args; break; \
      default : break;\
    }

///////////////////////////////////////////////////////////////////////////////


template<class T>
static T median_(const cv::Mat & image, cv::InputArray _mask)
{
  std::vector<T> vec;
  cv::Mat mask;
  int nnz;

  if ( (mask = _mask.getMat()).empty() ) {
    // spread Mat to single row
    if ( image.isContinuous() ) {
      image.reshape(0, 1).copyTo(vec);
    }
    else {
      cv::Mat array;
      image.copyTo(array);
      array.reshape(0, 1).copyTo(vec);
    }
  }
  else if ( (nnz = countNonZero(mask)) > 0 ) {
    vec.resize(nnz);
    for ( int y = 0, z = 0; y < mask.rows; ++y ) {
      const uint8_t * mp = mask.ptr<const uint8_t>(y);
      const T * imgp = image.ptr<const T>(y);
      for ( int x = 0; x < mask.cols; ++x ) {
        if ( mp[x] ) {
          vec[z++] = imgp[x];
        }
      }
    }
  }
  else {
    return 0;
  }

  // call median
  return median_(&(*vec.begin()), vec.size());
}



double median(const cv::Mat & image, cv::InputArray _mask)
{
  CV_Assert(!image.empty());
  CV_Assert(image.channels() == 1);

  CV_MTYPE_SWITCH_1(image, median_, (image, _mask));

  cv::error(cv::Error::BadDepth,
      "Unsupported image type",
      CV_Func,
      __FILE__,
      __LINE__);

  return 0;
}

