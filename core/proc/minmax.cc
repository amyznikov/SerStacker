/*
 * minmax.cc
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */
#include "minmax.h"
//#include <tbb/tbb.h>
#include <core/debug.h>

static inline void init_minmax(uint8_t * minv, uint8_t * maxv)
{
  *minv = UINT8_MAX;
  *maxv = 0;
}

static inline void init_minmax(int8_t * minv, int8_t * maxv)
{
  *minv = INT8_MAX;
  *maxv = INT8_MIN;
}

static inline void init_minmax(uint16_t * minv, uint16_t * maxv)
{
  *minv = UINT16_MAX;
  *maxv = 8;
}

static inline void init_minmax(int16_t * minv, int16_t * maxv)
{
  *minv = INT16_MAX;
  *maxv = INT16_MIN;
}

static inline void init_minmax(int32_t * minv, int32_t * maxv)
{
  *minv = INT32_MAX;
  *maxv = INT32_MIN;
}

static inline void init_minmax(float * minv, float * maxv)
{
  *minv = DBL_MAX;
  *maxv = -DBL_MAX;
}

static inline void init_minmax(double * minv, double * maxv)
{
  *minv = DBL_MAX;
  *maxv = -DBL_MAX;
}


template<class T>
static bool getminmax_(cv::InputArray _src, double * minval, double * maxval, cv::InputArray _mask)
{
  // TODO: implement with tbb

  const cv::Mat src = _src.getMat();
  const int cn = src.channels();

  T minv, maxv;

  init_minmax(&minv, &maxv);

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


static bool getminmax__(cv::InputArray src, double * minval, double * maxval, cv::InputArray mask)
{
  //if ( src.kind()


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

bool getminmax(cv::InputArray src, double * minval, double * maxval, cv::InputArray mask)
{
  if( !src.empty() ) {

    switch (src.kind()) {
      case cv::_InputArray::MAT:
        case cv::_InputArray::MATX:
        case cv::_InputArray::UMAT:
        case cv::_InputArray::CUDA_GPU_MAT:
        case cv::_InputArray::STD_VECTOR:
        case cv::_InputArray::STD_BOOL_VECTOR:
#if OPENCV_ABI_COMPATIBILITY < 500
        case cv::_InputArray::STD_ARRAY:
#endif
        return getminmax__(src, minval, maxval, mask);

      case cv::_InputArray::STD_VECTOR_VECTOR:
        case cv::_InputArray::STD_VECTOR_MAT:
        case cv::_InputArray::STD_VECTOR_UMAT:
        case cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT:
        case cv::_InputArray::STD_ARRAY_MAT: {

        double minv = *minval;
        double maxv = *maxval;

        const int nvecs =
            src.total();

        for( int i = 0; i < nvecs; ++i ) {
          getminmax__(src.getMat(i), &minv, &maxv, mask.empty() ? cv::noArray() : mask.getMat(i));
          if( minv < *minval ) {
            *minval = minv;
          }
          if( maxv > *maxval ) {
            *maxval = maxv;
          }
        }

        return true;
      }

    }
  }

  return false;
}



//bool getminmax(const std::vector<cv::Mat> &src, double * minval, double * maxval, const std::vector<cv::Mat> * masks)
//{
//  if ( !src.empty() ) {
//
//    getminmax(src[0], minval, maxval, masks && !masks->empty() ? (*masks)[0] : cv::noArray());
//
//    if ( src.size() > 1 ) {
//
//      double minv = *minval;
//      double maxv = *maxval;
//
//      for ( size_t i = 1, n = src.size(); i < n; ++i  ) {
//
//        getminmax(src[i], &minv, &maxv, masks && !masks->size() > i ? (*masks)[i] : cv::noArray());
//
//        if ( minv < *minval ) {
//          *minval = minv;
//        }
//        if ( maxv > *maxval ) {
//          *maxval = maxv;
//        }
//      }
//    }
//  }
//
//  return true;
//}


