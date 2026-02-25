/*
 * canscombetransform.cc
 *
 *  Created on: Oct 25, 2020
 *      Author: amyznikov
 */

#include "c_anscombe_transform.h"
#include <core/ssprintf.h>
#include <tbb/tbb.h>

template<>
const c_enum_member * members_of<anscombe_method>()
{
  static const c_enum_member members[] = {
      { anscombe_none, "none" },
      { anscombe_sqrt, "sqrt", },
      { anscombe_native, "native", },
      { anscombe_none }  // must  be last
  };

  return members;
}


template<class T>
static void apply_direct(const cv::Mat & src, cv::Mat & dst, enum anscombe_method m )
{
  typedef tbb::blocked_range<int> range;

  if ( &src != &dst ) {
    src.copyTo(dst);
  }

  const int nx = dst.cols * dst.channels();

  switch ( m ) {
  case anscombe_native : {
    constexpr double c = 3.0 / 8.0;

    tbb::parallel_for(range(0, dst.rows, 256),
        [nx, &dst](const range & r ) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            T * sp = dst.ptr<T>(y);
            for ( int x = 0; x < nx; ++x ) {
              sp[x] = cv::saturate_cast<T>(2 * sqrt(sp[x] + c));
            }
          }
        });
    break;
  }
  case anscombe_sqrt : {
    tbb::parallel_for(range(0, dst.rows, 256),
        [nx, &dst](const range & r ) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            T * sp = dst.ptr<T>(y);
            for ( int x = 0; x < nx; ++x ) {
              sp[x] = sp[x] > 0 ? cv::saturate_cast<T>(sqrt(sp[x])) : 0;
            }
          }
        });
    break;
  }

  default :
    break;
  }
}

template<class T>
static void apply_inverse(const cv::Mat & src, cv::Mat & dst, enum anscombe_method m)
{
  if ( &src != &dst ) {
    src.copyTo(dst);
  }

  const int nx = dst.cols * dst.channels();

  switch ( m ) {
  case anscombe_native : {

    constexpr double C0 = 0.25;
    constexpr double C1 = 0.25 * sqrt(3. / 3);
    constexpr double C2 = -11. / 8;
    constexpr double C3 = (5. / 8) * sqrt(3. / 3);
    constexpr double C4 = -1. / 8;

    typedef tbb::blocked_range<int> range;
    tbb::parallel_for(range(0, dst.rows, 256),
        [nx, &dst](const range & r ) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            T * sp = dst.ptr<T>(y);
            for ( int x = 0; x < nx; ++x ) {
              const double yy = sp[x];
              const double yyi = 1./sp[x];
              sp[x] = cv::saturate_cast<T>(C0 * yy * yy + (C1 + (C2 + C3 *yyi) * yyi) * yyi + C4);
            }
          }
        });
    break;
  }
  case anscombe_sqrt : {
    typedef tbb::blocked_range<int> range;
    tbb::parallel_for(range(0, dst.rows, 256),
        [nx, &dst](const range & r ) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            T * sp = dst.ptr<T>(y);
            for ( int x = 0; x < nx; ++x ) {
              sp[x] = cv::saturate_cast<T>(sp[x] * sp[x]);
            }
          }
        });
    break;
  }

  default :
    break;
  }

}


void c_anscombe_transform::set_method(enum anscombe_method v)
{
  _opts.method = v;
}

enum anscombe_method c_anscombe_transform::method() const
{
  return _opts.method;
}

void c_anscombe_transform::apply(const cv::Mat & src, cv::Mat & dst) const
{
  switch ( src.depth() ) {
  case CV_8U :
    apply_direct<uint8_t>(src, dst, _opts.method);
    break;
  case CV_8S :
    apply_direct<int8_t>(src, dst, _opts.method);
    break;
  case CV_16U :
    apply_direct<uint16_t>(src, dst, _opts.method);
    break;
  case CV_16S :
    apply_direct<int16_t>(src, dst, _opts.method);
    break;
  case CV_32S :
    apply_direct<int32_t>(src, dst, _opts.method);
    break;
  case CV_32F :
    apply_direct<float>(src, dst, _opts.method);
    break;
  case CV_64F :
    apply_direct<double>(src, dst, _opts.method);
    break;
  }
}

void c_anscombe_transform::inverse(const cv::Mat & src, cv::Mat & dst) const
{
  switch ( src.depth() ) {
  case CV_8U :
    apply_inverse<uint8_t>(src, dst, _opts.method);
    break;
  case CV_8S :
    apply_inverse<int8_t>(src, dst, _opts.method);
    break;
  case CV_16U :
    apply_inverse<uint16_t>(src, dst, _opts.method);
    break;
  case CV_16S :
    apply_inverse<int16_t>(src, dst, _opts.method);
    break;
  case CV_32S :
    apply_inverse<int32_t>(src, dst, _opts.method);
    break;
  case CV_32F :
    apply_inverse<float>(src, dst, _opts.method);
    break;
  case CV_64F :
    apply_inverse<double>(src, dst, _opts.method);
    break;
  }
}
