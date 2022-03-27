/*
 * c_display_function.cc
 *
 *  Created on: Dec 14, 2020
 *      Author: amyznikov
 */

#include "c_display_function.h"
#include <core/debug.h>


c_image_display_function::c_image_display_function()
  : mtf_(c_pixinsight_midtones_transfer_function::create())
{
}


const c_pixinsight_midtones_transfer_function::sptr & c_image_display_function::mtf() const
{
  return mtf_;
}

void c_image_display_function::operator () (const cv::Mat & src, cv::Mat & dst, int ddepth) const
{
  mtf_->apply(src, dst, ddepth);
}
