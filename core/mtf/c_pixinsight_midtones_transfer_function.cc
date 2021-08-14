/*
 * c_pixinsight_midtones_transfer_function.cc
 *
 *  Created on: Dec 16, 2020
 *      Author: amyznikov
 */

#include "c_pixinsight_midtones_transfer_function.h"
#include "pixinsight-mtf.h"


c_pixinsight_midtones_transfer_function::ptr c_pixinsight_midtones_transfer_function::create()
{
  return ptr(new c_pixinsight_midtones_transfer_function());
}


void c_pixinsight_midtones_transfer_function::set_midtones(double v)
{
  midtones_ = v;
}

double c_pixinsight_midtones_transfer_function::midtones() const
{
  return midtones_;
}

bool c_pixinsight_midtones_transfer_function::apply(cv::InputArray src_image,
    cv::OutputArray dst_image,
    int ddepth) const
{
  return apply_mtf_pixinsight(src_image,
      dst_image,
      input_range_[0], input_range_[1],
      output_range_[0], output_range_[1],
      shadows_,
      highlights_,
      midtones_,
      ddepth);
}

bool c_pixinsight_midtones_transfer_function::find_midtones_balance(cv::InputArray input_image_histogram,
    double * output_shadows, double * output_highlights, double * output_midtones)
{
  return find_midtones_balance_pixinsight(input_image_histogram,
      output_shadows, output_highlights, output_midtones);
}

bool c_pixinsight_midtones_transfer_function::find_midtones_balance(cv::InputArray input_image_histogram)
{
  return this_class::find_midtones_balance(input_image_histogram,
      &shadows_, &highlights_, &midtones_);
}

