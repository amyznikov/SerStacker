/*
 * minmax.cc
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */
#include "minmax.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>

#if HAVE_TBB
#include <tbb/tbb.h>

namespace {

struct MinMaxReducer
{
  const cv::Mat & src;
  const cv::Mat & mask;
  const int cn;
  double minVal, maxVal;
  bool found;

  MinMaxReducer(const cv::Mat & s, const cv::Mat & m, int c) :
    src(s), mask(m), cn(c), minVal(DBL_MAX), maxVal(-DBL_MAX), found(false)
  {
  }

  MinMaxReducer(MinMaxReducer & x, tbb::split) :
    src(x.src), mask(x.mask), cn(x.cn), minVal(DBL_MAX), maxVal(-DBL_MAX), found(false)
  {
  }

  template<typename T>
  void process(const tbb::blocked_range2d<int> & r)
  {
    const int _cn = cn;
    const auto & rows = r.rows();
    const auto & cols = r.cols();

    double l_min = minVal;
    double l_max = maxVal;
    bool l_found = found;

    if( mask.empty() ) {
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          const T * pxp = srcp + x * _cn;
          for( int c = 0; c < _cn; ++c ) {
            if( std::isfinite(pxp[c]) ) {
              const double v = pxp[c];
              if( v < l_min ) {
                l_min = v;
              }
              if( v > l_max ) {
                l_max = v;
              }
              l_found = true;
            }
          }
        }
      }
    }
    else if( mask.channels() == 1 ) {
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          if( mskp[x] ) {
            const T * pxp = srcp + x * _cn;
            for( int c = 0; c < _cn; ++c ) {
              if( std::isfinite(pxp[c]) ) {
                const double v = pxp[c];
                if( v < l_min ) {
                  l_min = v;
                }
                if( v > l_max ) {
                  l_max = v;
                }
                l_found = true;
              }
            }
          }
        }
      }
    }
    else { // mask.channels() == _cn
      for( int y = rows.begin(); y < rows.end(); ++y ) {
        const T * srcp = src.ptr<T>(y);
        const uint8_t * mskp = mask.ptr<uint8_t>(y);
        for( int x = cols.begin(); x < cols.end(); ++x ) {
          const T * pxp = srcp + x * _cn;
          const uint8_t * mpxp = mskp + x * _cn;
          for( int c = 0; c < _cn; ++c ) {
            if( mpxp[c] && std::isfinite(pxp[c])  ) {
              const double v = pxp[c];
              if( v < l_min ) {
                l_min = v;
              }
              if( v > l_max ) {
                l_max = v;
              }
              l_found = true;
            }
          }
        }
      }
    }

    minVal = l_min;
    maxVal = l_max;
    found = l_found;
  }

  void operator()(const tbb::blocked_range2d<int> & r)
  {
    CV_DISPATCH(src.depth(), process, r);
  }

  void join(const MinMaxReducer & y)
  {
    if( y.found ) {
      if( y.minVal < minVal ) {
        minVal = y.minVal;
      }
      if( y.maxVal > maxVal ) {
        maxVal = y.maxVal;
      }
      found = true;
    }
  }
};
} // namespace

#else // HAVE_TBB

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
static bool _getminmax(cv::InputArray _src, double * minval, double * maxval, cv::InputArray _mask)
{
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
  else if( cn > 1 && _mask.channels() == cn ) {
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
    cv::Mat1b mask;
    if( _mask.channels() == 1  )  {
      mask = _mask.getMat();
    }
    else {
      reduce_color_channels(_mask.getMat(), mask, cv::REDUCE_MAX);
    }

    if( cn == 1 ) {
      for( int y = 0; y < src.rows; ++y ) {
        const T *srcp = src.ptr<const T>(y);
        const uint8_t *mskp = mask[y];
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
        const uint8_t *mskp = mask[y];
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

  *minval = minv;
  *maxval = maxv;

  return true;
}

#endif // HAVE_TBB

static bool __getminmax(cv::InputArray _src, double * minval, double * maxval, cv::InputArray _mask)
{
#if !HAVE_TBB
  CV_DISPATCH(_src.depth(), _getminmax, _src, minval, maxval, mask);
  CF_FATAL("Invalid argument: Unsupported src depth : %d", _src.depth());
  return false;
#else
  const cv::Mat src = _src.getMat();
  const cv::Mat mask = _mask.getMat();

  MinMaxReducer reducer(src, mask, _src.channels());
  tbb::parallel_reduce(tbb::blocked_range2d<int>(0, _src.rows(), 0, _src.cols()), reducer,
      tbb::static_partitioner());

  if( reducer.found ) {
    *minval = reducer.minVal;
    *maxval = reducer.maxVal;
  }

  return reducer.found;
#endif
}

bool getMinMax(cv::InputArray src, double * minval, double * maxval, cv::InputArray mask)
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
        return __getminmax(src, minval, maxval, mask);

      case cv::_InputArray::STD_VECTOR_VECTOR:
        case cv::_InputArray::STD_VECTOR_MAT:
        case cv::_InputArray::STD_VECTOR_UMAT:
        case cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT:
        case cv::_InputArray::STD_ARRAY_MAT: {

        double minv = *minval;
        double maxv = *maxval;

        const int nvecs = src.total();
        for( int i = 0; i < nvecs; ++i ) {
          __getminmax(src.getMat(i), &minv, &maxv, mask.empty() ? cv::noArray() : mask.getMat(i));
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



