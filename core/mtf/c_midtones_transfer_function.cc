/*
 * c_midtones_transfer_function.cc
 *
 *  Created on: Dec 16, 2020
 *      Author: amyznikov
 */

#include "c_midtones_transfer_function.h"
#include <tbb/tbb.h>
#include <core/debug.h>


template<class T1, class T2>
static bool apply__(const cv::Mat & src, cv::Mat & dst,
    double imin, double imax,
    double omin, double omax)
{
  using tbb_range = tbb::blocked_range<int>;

  const int ny = src.rows;
  const int nx = src.cols * src.channels();
  const double s = (omax - omin) / (imax - imin);

  tbb::parallel_for(tbb_range(0, ny, 256),
      [&src, &dst, nx, imin, omin, s]

      (const tbb_range & r) {

        for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

          const T1 * srcp = src.ptr<const T1>(y);
          T2 * dstp = dst.ptr<T2>(y);

          for ( int x = 0; x < nx; ++x ) {
            dstp[x] = cv::saturate_cast<T2>(omin + (srcp[x] - imin) * s);
          }
        }
      });

  return true;
}

template<class T1>
static bool apply_(const cv::Mat & src, cv::Mat & dst,
    double imin, double imax,
    double omin, double omax)
{
  switch ( dst.depth() ) {
  case CV_8U :
    return apply__<T1, uint8_t>(src, dst, imin, imax, omin, omax);
  case CV_8S :
    return apply__<T1, int8_t>(src, dst, imin, imax, omin, omax);
  case CV_16U :
    return apply__<T1, uint16_t>(src, dst, imin, imax, omin, omax);
  case CV_16S :
    return apply__<T1, int16_t>(src, dst, imin, imax, omin, omax);
  case CV_32S :
    return apply__<T1, int32_t>(src, dst, imin, imax, omin, omax);
  case CV_32F :
    return apply__<T1, float>(src, dst, imin, imax, omin, omax);
  case CV_64F :
    return apply__<T1, double>(src, dst, imin, imax, omin, omax);
  default:
    break;
  }

  return false;
}


c_midtones_transfer_function::ptr c_midtones_transfer_function::create()
{
  return ptr(new c_midtones_transfer_function());
}

void c_midtones_transfer_function::set_input_range(double minval, double maxval)
{
  input_range_[0] = minval;
  input_range_[1] = maxval;
}

void c_midtones_transfer_function::get_input_range(double * minval, double  * maxval) const
{
  * minval = input_range_[0];
  * maxval = input_range_[1];
}

void c_midtones_transfer_function::set_output_range(double minval, double maxval)
{
  output_range_[0] = minval;
  output_range_[1] = maxval;
}

void c_midtones_transfer_function::get_output_range(double * minval, double * maxval) const
{
  *minval = output_range_[0];
  *maxval = output_range_[1];
}

void c_midtones_transfer_function::set_shadows(double v)
{
  shadows_ = v;
}

double c_midtones_transfer_function::shadows() const
{
  return shadows_;
}

void c_midtones_transfer_function::set_highlights(double v)
{
  highlights_ = v;
}

double c_midtones_transfer_function::highlights() const
{
  return highlights_;
}

bool c_midtones_transfer_function::suggest_levels_range(int depth, double * minval, double * maxval)
{
  switch ( depth ) {
  case CV_8U :
    *minval = 0;
    *maxval = UINT8_MAX;
    break;
  case CV_8S :
    *minval = INT8_MIN;
    *maxval = INT8_MAX;
    break;
  case CV_16U :
    *minval = 0;
    *maxval = UINT16_MAX;
    break;
  case CV_16S :
    *minval = INT16_MIN;
    *maxval = INT16_MAX;
    break;
  case CV_32S :
    *minval = INT32_MIN;
    *maxval = INT32_MAX;
    break;
  case CV_32F :
    *minval = 0;
    *maxval = 1;
    break;
  case CV_64F :
    *minval = 0;
    *maxval = 1;
    break;
  default:
    *minval = 0;
    *maxval = 1;
    return false;
  }

  return true;
}

double c_midtones_transfer_function::apply(double pix) const
{
  double imin = input_range_[0];
  double imax = input_range_[1];
  double omin = output_range_[0];
  double omax = output_range_[1];
  if ( imin >= imax ) {
    imin = 0;
    imax  = 1;
  }
  if ( omin >= omax ) {
    omin = 0;
    omax  = 1;
  }

  return omin + (pix - imin) * (omax - omin) / (imax - imin);
}

bool c_midtones_transfer_function::apply(cv::InputArray src_image,
    cv::OutputArray dst_image,
    int ddepth ) const
{
  if ( !dst_image.needed() ) {
    return false;
  }

  if ( dst_image.fixedType() ) {
    ddepth = dst_image.depth();
  }
  else if ( ddepth < CV_8U || ddepth > CV_64F ) {
    ddepth = src_image.depth();
  }


  double src_min, src_max;
  double dst_min, dst_max;
  double imin, imax, omin, omax;
  cv::Mat src, dst;

  src = src_image.getMat();

  if ( src.data != dst_image.getMat().data ) {
    // create external storage for output
    dst_image.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));
    dst = dst_image.getMatRef();
  }
  else if ( src.depth() == ddepth ) {
    // process in place
    dst = src;
  }
  else {
    // create internal storage for output
    dst.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));
  }


  if ( input_range_[0] >= input_range_[1] ) {
    suggest_levels_range(src.depth(),
        &src_min, &src_max);
  }
  else {
    src_min = input_range_[0];
    src_max = input_range_[1];
  }


  if ( output_range_[0] >= output_range_[1] ) {
    suggest_levels_range(ddepth,
        &dst_min, &dst_max);
  }
  else {
    dst_min = output_range_[0];
    dst_max = output_range_[1];
  }

  imin = src_min + shadows_ * (src_max - src_min);
  imax = src_min + highlights_ * (src_max - src_min);
  omin = dst_min;
  omax = dst_max;

  switch ( src.depth() ) {
  case CV_8U :
    apply_<uint8_t>(src, dst, imin, imax, omin, omax);
    break;
  case CV_8S :
    apply_<int8_t>(src, dst, imin, imax, omin, omax);
    break;
  case CV_16U :
    apply_<uint16_t>(src, dst, imin, imax, omin, omax);
    break;
  case CV_16S :
    apply_<int16_t>(src, dst, imin, imax, omin, omax);
    break;
  case CV_32S :
    apply_<int32_t>(src, dst, imin, imax, omin, omax);
    break;
  case CV_32F :
    apply_<float>(src, dst, imin, imax, omin, omax);
    break;
  case CV_64F :
    apply_<double>(src, dst, imin, imax, omin, omax);
    break;
  default:
    break;
  }

  if ( dst.data != dst_image.getMatRef().data ) {
    dst_image.move(dst);
  }

  return true;
}

