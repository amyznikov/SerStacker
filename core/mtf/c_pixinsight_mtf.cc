/*
 * c_pixinsight_mtf.cc
 *
 *  Created on: Dec 16, 2020
 *      Author: amyznikov
 */

#include "c_pixinsight_mtf.h"
#include "pixinsight-mtf.h"
#include <core/debug.h>


c_pixinsight_mtf::sptr c_pixinsight_mtf::create()
{
  return sptr(new c_pixinsight_mtf());
}


void c_pixinsight_mtf::set_midtones(double v)
{
  midtones_ = v;
}

double c_pixinsight_mtf::midtones() const
{
  return midtones_;
}

void c_pixinsight_mtf::set_parameters(double shadows, double highlights, double midtones)
{
  shadows_ = shadows;
  highlights_ = highlights;
  midtones_ = midtones;
}

void c_pixinsight_mtf::get_parameters(double * shadows, double * highlights, double * midtones) const
{
  *shadows = shadows_;
  *highlights = highlights_;
  *midtones = midtones_;
}

double c_pixinsight_mtf::apply(double pix) const
{
  const double & srcmin = input_range_[0];
  const double & srcmax = input_range_[1];
  const double & dstmin = output_range_[0];
  const double & dstmax = output_range_[1];

//  if ( srcmin >= srcmax ) {
//    srcmin = 0;
//    srcmax  = 1;
//  }
//  if ( dstmin >= dstmax ) {
//    dstmin = srcmin;
//    dstmax  = srcmax;
//  }

  const double imin = srcmin + shadows_ * (srcmax - srcmin);
  const double imax = srcmin + highlights_ * (srcmax - srcmin);
  const double si = 1. / (imax - imin);
  const double so = (dstmax - dstmin);

  return dstmin + so * mtf_pixinsight((pix - imin) * si, midtones_);

}

bool c_pixinsight_mtf::apply(cv::InputArray src_image,
    cv::OutputArray dst_image,
    int ddepth) const
{

  if( fabs(midtones_ - 0.5) < 10 * FLT_EPSILON ) {
    return base::apply(src_image, dst_image, ddepth);
  }

  return apply_mtf_pixinsight(src_image,
      dst_image,
      input_range_[0], input_range_[1],
      output_range_[0], output_range_[1],
      shadows_,
      highlights_,
      midtones_,
      ddepth);
}

bool c_pixinsight_mtf::find_midtones_balance(cv::InputArray input_image_histogram,
    double * output_shadows, double * output_highlights, double * output_midtones)
{
  return find_midtones_balance_pixinsight(input_image_histogram,
      output_shadows, output_highlights, output_midtones);
}

bool c_pixinsight_mtf::find_midtones_balance(cv::InputArray input_image_histogram)
{
  return this_class::find_midtones_balance(input_image_histogram,
      &shadows_, &highlights_, &midtones_);
}

