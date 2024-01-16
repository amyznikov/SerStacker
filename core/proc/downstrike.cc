/*
 * downstrike.cc
 *
 *  Created on: May 26, 2018
 *      Author: amyznikov
 */
#include "downstrike.h"
#include <core/ssprintf.h>
#include <tbb/tbb.h>
#include <core/debug.h>



template<>
const c_enum_member* members_of<DOWNSTRIKE_MODE>()
{
  static const c_enum_member members[] = {
      { DOWNSTRIKE_EVEN, "DOWNSTRIKE_EVEN", "Reject each EVEN row and column: 0,2,4,6..." },
      { DOWNSTRIKE_UNEVEN, "DOWNSTRIKE_UNEVEN", "Reject each UNEVEN row and column: 1,3,5,7 ..." },
      { DOWNSTRIKE_UNEVEN},
  };

  return members;
}


///////////////////////////////////////////////////////////////////////////////
using namespace cv;


/*
 * for single-channel images
 *  REMOVE each EVEN row and column,
 *    keep only uneven
 * */
template<class T>
static void downstrike_even_1c(const cv::Mat & src, cv::Mat & _dst, cv::Size dst_size)
{
  cv::Mat tmp;

  if( dst_size.empty() ) {
    dst_size = cv::Size((src.cols + 1) / 2, (src.rows + 1) / 2);
  }


  cv::Mat &dst =
      src.data == _dst.data ? tmp :
          _dst;

  dst.create(dst_size, src.type());
  dst.setTo(0);

  const int ymax = 2 * (dst.rows - 1) + 1 < src.rows ? dst.rows : dst.rows - 1;
  const int xmax = 2 * (dst.cols - 1) + 1 < src.cols ? dst.cols : dst.cols - 1;

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, ymax, 64),
      [&src, &dst, xmax] (const range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          const T * srcp = src.ptr<const T>(2 * y + 1);
          T * dstp = dst.ptr<T>(y);
          for ( int x = 0; x < xmax; ++x ) {
            dstp[x] = srcp[2 * x + 1];
          }
        }
      });

  if ( dst.data != _dst.data ) {
    _dst = std::move(dst);
  }
}


/*
 * for single-channel images
 *  REMOVE each UNEVEN (ODD) row and column,
 *    keep only even
 * */
template<class T>
static void downstrike_uneven_1c(const cv::Mat & src, cv::Mat & _dst, cv::Size dst_size)
{
  cv::Mat tmp;

  if( dst_size.empty() ) {
    dst_size = cv::Size((src.cols + 1) / 2, (src.rows + 1) / 2);
  }

  cv::Mat & dst = src.data == _dst.data ? tmp : _dst;
  dst.create(dst_size, src.type());
  dst.setTo(0);

  const int ymax = dst.rows;
  const int xmax = dst.cols;

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, ymax, 64),
      [&src, &dst, xmax] (const range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          const T * srcp = src.ptr<const T>(2 * y);
          T * dstp = dst.ptr<T>(y);
          for ( int x = 0; x < xmax; ++x ) {
            dstp[x] = srcp[2 * x];
          }
        }
      });

  if ( dst.data != _dst.data ) {
    _dst = std::move(dst);
  }
}




/*
 * for multi-channel images
 *  REMOVE each EVEN row and column,
 *    keep only uneven
 * */
static void downstrike_even_mc(const cv::Mat & src, cv::Mat & _dst, cv::Size dst_size)
{
  if( dst_size.empty() ) {
    dst_size = cv::Size((src.cols + 1) / 2, (src.rows + 1) / 2);
  }

  cv::Mat tmp;
  cv::Mat & dst = src.data == _dst.data ? tmp : _dst;

  dst.create(dst_size, src.type());
  dst.setTo(0);

  const int ymax = 2 * (dst.rows - 1) + 1 < src.rows ? dst.rows : dst.rows - 1;
  const int xmax = 2 * (dst.cols - 1) + 1 < src.cols ? dst.cols : dst.cols - 1;
  const size_t elem_size = src.elemSize();

  for ( int y = 0; y < ymax; ++y ) {
    const uint8_t * s = src.ptr(2 * y + 1);
    uint8_t * d = dst.ptr(y);
    for ( int x = 0; x < xmax; ++x ) {
      memcpy(d + x * elem_size, s + (2 * x + 1) * elem_size, elem_size);
    }
  }

  if ( dst.data != _dst.data ) {
    _dst = std::move(dst);
  }
}



/*
 * for multi-channel images
 *  REMOVE each UNEVEN (ODD) row and column,
 *    keep only even
 * */
static void downstrike_uneven_mc(const cv::Mat & src, cv::Mat & _dst, cv::Size dst_size)
{
  cv::Mat tmp;

  if( dst_size.empty() ) {
    dst_size = cv::Size((src.cols + 1) / 2, (src.rows + 1) / 2);
  }

  cv::Mat & dst =
      src.data == _dst.data ? tmp :
          _dst;

  dst.create(dst_size, src.type());
  dst.setTo(0);

  const int ymax = dst.rows;
  const int xmax = dst.cols;
  const size_t elem_size = src.elemSize();

  for ( int y = 0; y < ymax; ++y ) {
    const uint8_t * s = src.ptr(2 * y);
    uint8_t * d = dst.ptr(y);
    for ( int x = 0; x < xmax; ++x ) {
      memcpy(d + x * elem_size, s + 2 * x * elem_size, elem_size);
    }
  }

  if ( dst.data != _dst.data ) {
    _dst = std::move(dst);
  }
}





/*
 * 2x downsampling step by rejecting each EVEN row and column,
 *    keep only uneven
 * */
void downstrike_even(cv::InputArray _src, cv::Mat & dst, const cv::Size & size)
{
  INSTRUMENT_REGION("");

 const Mat src =
     _src.getMat();

  if ( src.channels() > 1 ) {
    downstrike_even_mc(src, dst, size);
  }
  else {
    switch ( src.type() ) {
    case CV_8U:
      return downstrike_even_1c<uint8_t>(src, dst, size);
    case CV_8S:
      return downstrike_even_1c<int8_t>(src, dst, size);
    case CV_16U:
      return downstrike_even_1c<uint16_t>(src, dst, size);
    case CV_16S:
      return downstrike_even_1c<int16_t>(src, dst, size);
    case CV_32F:
      return downstrike_even_1c<float>(src, dst, size);
    case CV_64F:
      return downstrike_even_1c<double>(src, dst, size);
    default:
      return downstrike_even_mc(src, dst, size);
    }
  }
}


/*
 * 2x downsampling step by rejecting each UNEVEN (ODD) row and column,
 *    keep only even
 * */
void downstrike_uneven(cv::InputArray _src, cv::Mat & dst, const cv::Size & size)
{
  const Mat src = _src.getMat();

  if ( src.channels() > 1 ) {
    downstrike_uneven_mc(src, dst, size);
  }
  else {
    switch ( src.type() ) {
    case CV_8U:
      return downstrike_uneven_1c<uint8_t>(src, dst, size);
    case CV_8S:
      return downstrike_uneven_1c<int8_t>(src, dst, size);
    case CV_16U:
      return downstrike_uneven_1c<uint16_t>(src, dst, size);
    case CV_16S:
      return downstrike_uneven_1c<int16_t>(src, dst, size);
    case CV_32F:
      return downstrike_uneven_1c<float>(src, dst, size);
    case CV_64F:
      return downstrike_uneven_1c<double>(src, dst, size);
    default:
      return downstrike_uneven_mc(src, dst, size);
    }
  }
}



/*
 * 2x upsampling step by injecting EVEN ZERO rows and columns ...
 * */
template<class T>
static void upject_even_1c(const Mat & src, Mat & _dst, Size dstSize, Mat * _zmask = nullptr, int zmdepth = -1)
{
  cv::Mat tmp;
  cv::Mat1b zmask;

  cv::Mat & dst = src.data == _dst.data ? tmp : _dst;

  dst.create(dstSize, src.type());
  dst.setTo(0);

  if ( _zmask ) {
    zmask = Mat::zeros(dstSize, CV_8U);
  }


  const int ymax = 2 * (src.rows - 1) + 1 < dst.rows ? src.rows : src.rows - 1;
  const int xmax = 2 * (src.cols - 1) + 1 < dst.cols ? src.cols : src.cols - 1;


  typedef tbb::blocked_range<int> range;


  if ( !_zmask ) {

    tbb::parallel_for(range(0, ymax, 64),
        [&src, &dst, xmax](const range & r ) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            const T * srcp = src.ptr<const T>(y);
            T * dstp = dst.ptr<T>(2 * y + 1);
            for ( int x = 0; x < xmax; ++x ) {
              dstp[2 * x + 1] = srcp[x];
            }
          }
        });
  }
  else {
    tbb::parallel_for(range(0, ymax, 64),
        [&src, &dst, &zmask, xmax](const range & r ) {
      for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
        const T * srcp = src.ptr<const T>(y);
        T * dstp = dst.ptr<T>(2 * y + 1);
        uint8_t * mskp = zmask.ptr<uint8_t>(2 * y + 1);
        for ( int x = 0; x < xmax; ++x ) {
          dstp[2 * x + 1] = srcp[x];
          mskp[2 * x + 1] = 255;
        }
      }
    });
  }

  if ( _dst.data != dst.data ) {
    _dst = std::move(dst);
  }

  if ( _zmask ) {
    if ( CV_MAT_DEPTH(zmdepth) == CV_8U ) {
      *_zmask = std::move(zmask);
    }
    else {
      zmask.convertTo(*_zmask, CV_MAT_DEPTH(zmdepth), 1.0 / 255);
    }
  }
}



/*
 * 2x upsampling step by injecting EVEN ZERO rows and columns ...
 * */
static void upject_even_mc(const Mat & src, Mat & _dst, Size dstSize, Mat * _zmask = NULL, int zmdepth=-1)
{
  Mat dst, zmask;
  const uint8_t * srcp;
  uint8_t * dstp;
  uint8_t * mskp;

  dst.create(dstSize, src.type());
  dst.setTo(0);

  if ( _zmask ) {
    zmask =
        Mat::zeros(dstSize, CV_8U);
  }

  const size_t elem_size =
      src.elemSize();

  const int ymax =
      2 * (src.rows - 1) + 1 < dst.rows ?
          src.rows :
          src.rows - 1;

  const int xmax =
      2 * (src.cols - 1) + 1 < dst.cols ?
          src.cols :
          src.cols - 1;

  if ( !_zmask ) {
    for ( int y = 0; y < ymax; ++y ) {
      srcp = src.ptr(y);
      dstp = dst.ptr(2 * y + 1);
      for ( int x = 0; x < xmax; ++x ) {
        memcpy(dstp + (2 * x + 1) * elem_size, srcp + x * elem_size, elem_size);
      }
    }
  }
  else {
    for ( int y = 0; y < ymax; ++y ) {
      srcp = src.ptr(y);
      dstp = dst.ptr(2 * y + 1);
      mskp = zmask.ptr<uint8_t>(2 * y + 1);
      for ( int x = 0; x < xmax; ++x ) {
        memcpy(dstp + (2 * x + 1) * elem_size, srcp + x * elem_size, elem_size);
        mskp[2 * x + 1] = 255;
      }
    }
  }

  _dst = dst;

  if ( _zmask ) {
    if ( CV_MAT_DEPTH(zmdepth) == CV_8U ) {
      *_zmask = zmask;
    }
    else {
      zmask.convertTo(*_zmask, CV_MAT_DEPTH(zmdepth), 1.0 / 255);
    }
  }
}

/*
 * 2x upsampling step by injecting UNEVEN ZERO rows and columns ...
 * */
template<class T>
static void upject_uneven_1c(const Mat & src, Mat & _dst, Size dstSize, Mat * _zmask = nullptr, int zmdepth=-1)
{
  cv::Mat tmp;
  cv::Mat1b zmask;

  cv::Mat & dst = src.data == _dst.data ? tmp : _dst;

  dst.create(dstSize, src.type());
  dst.setTo(0);

  if ( _zmask ) {
    zmask = Mat::zeros(dstSize, CV_8U);
  }


  const int ymax = 2 * (src.rows - 1) < dst.rows ? src.rows : src.rows - 1;
  const int xmax = 2 * (src.cols - 1) < dst.cols ? src.cols : src.cols - 1;

  typedef tbb::blocked_range<int> range;

  if ( !_zmask ) {

    tbb::parallel_for(range(0, ymax, 64),
        [&src, &dst, xmax](const range & r ) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            const T * srcp = src.ptr<const T>(y);
            T * dstp = dst.ptr<T>(2 * y);
            for ( int x = 0; x < xmax; ++x ) {
              dstp[2 * x] = srcp[x];
            }
          }
        });
  }
  else {
    tbb::parallel_for(range(0, ymax, 64),
        [&src, &dst, &zmask, xmax](const range & r ) {
      for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
        const T * srcp = src.ptr<const T>(y);
        T * dstp = dst.ptr<T>(2 * y);
        uint8_t * mskp = zmask.ptr<uint8_t>(2 * y);
        for ( int x = 0; x < xmax; ++x ) {
          dstp[2 * x] = srcp[x];
          mskp[2 * x] = 255;
        }
      }
    });
  }

  if ( _dst.data != dst.data ) {
    _dst = std::move(dst);
  }

  if ( _zmask ) {
    if ( CV_MAT_DEPTH(zmdepth) == CV_8U ) {
      *_zmask = std::move(zmask);
    }
    else {
      zmask.convertTo(*_zmask, CV_MAT_DEPTH(zmdepth), 1.0 / 255);
    }
  }
}




/*
 * 2x upsampling step by injecting UNEVEN ZERO rows and columns ...
 * */
static void upject_uneven_mc(const Mat & src, Mat & _dst, Size dstSize, Mat * _zmask = NULL, int zmdepth=-1)
{
  Mat dst, zmask;
  const uint8_t * srcp;
  uint8_t * dstp;
  uint8_t * mskp;

  dst.create(dstSize, src.type());
  dst.setTo(0);

  if ( _zmask ) {
    zmask = Mat::zeros(dstSize, CV_8U);
  }

  const size_t elem_size = src.elemSize();

  const int ymax = 2 * (src.rows - 1) < dst.rows ? src.rows : src.rows - 1;
  const int xmax = 2 * (src.cols - 1) < dst.cols ? src.cols : src.cols - 1;

  if ( !_zmask ) {
    for ( int y = 0; y < ymax; ++y ) {
      srcp = src.ptr(y);
      dstp = dst.ptr(2 * y);
      for ( int x = 0; x < xmax; ++x ) {
        memcpy(dstp + 2 * x * elem_size, srcp + x * elem_size, elem_size);
      }
    }
  }
  else {
    for ( int y = 0; y < ymax; ++y ) {
      srcp = src.ptr(y);
      dstp = dst.ptr(2 * y);
      mskp = zmask.ptr<uint8_t>(2 * y);
      for ( int x = 0; x < xmax; ++x ) {
        memcpy(dstp + 2 * x * elem_size, srcp + x * elem_size, elem_size);
        mskp[2 * x] = 255;
      }
    }
  }

  _dst = dst;

  if ( _zmask ) {
    if ( CV_MAT_DEPTH(zmdepth) == CV_8U ) {
      *_zmask = zmask;
    }
    else {
      zmask.convertTo(*_zmask, CV_MAT_DEPTH(zmdepth), 1.0 / 255);
    }
  }
}







/*
 * 2x upsampling step by injecting EVEN ZERO rows and columns ...
 * */
void upject_even(cv::InputArray _src, Mat & dst, cv::Size dstSize, Mat * _zmask, int zmdepth)
{
  INSTRUMENT_REGION("");

  const Mat src =
      _src.getMat();

  if ( dstSize.empty() ) {
    dstSize = src.size() * 2;
  }

  if ( src.channels() > 1 ) {
    upject_even_mc(src, dst, dstSize, _zmask, zmdepth);
  }
  else {
    switch ( src.type() ) {
    case CV_8U:
      return upject_even_1c<uint8_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_8S:
      return upject_even_1c<int8_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_16U:
      return upject_even_1c<uint16_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_16S:
      return upject_even_1c<int16_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_32F:
      return upject_even_1c<float>(src, dst, dstSize, _zmask, zmdepth);
    case CV_64F:
      return upject_even_1c<double>(src, dst, dstSize, _zmask, zmdepth);
    default:
      return upject_even_mc(src, dst, dstSize, _zmask, zmdepth);
    }
  }
}


/*
 * 2x upsampling step by injecting UNEVEN ZERO rows and columns ...
 * */
void upject_uneven(cv::InputArray _src, Mat & dst, Size dstSize, Mat * _zmask, int zmdepth)
{
  const Mat src = _src.getMat();

  if ( dstSize.empty() ) {
    dstSize = src.size() * 2;
  }

  if ( src.channels() > 1 ) {
    upject_uneven_mc(src, dst, dstSize, _zmask, zmdepth);
  }
  else {
    switch ( src.type() ) {
    case CV_8U:
      return upject_uneven_1c<uint8_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_8S:
      return upject_uneven_1c<int8_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_16U:
      return upject_uneven_1c<uint16_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_16S:
      return upject_uneven_1c<int16_t>(src, dst, dstSize, _zmask, zmdepth);
    case CV_32F:
      return upject_uneven_1c<float>(src, dst, dstSize, _zmask, zmdepth);
    case CV_64F:
      return upject_uneven_1c<double>(src, dst, dstSize, _zmask, zmdepth);
    default:
      return upject_uneven_mc(src, dst, dstSize, _zmask, zmdepth);
    }
  }
}



///////////////////////////////////////////////////////////////////////////////
