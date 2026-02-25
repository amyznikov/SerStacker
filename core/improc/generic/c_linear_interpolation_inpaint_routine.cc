/*
 * c_linear_interpolation_inpaint_routine.cc
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#include "c_linear_interpolation_inpaint_routine.h"

void c_linear_interpolation_inpaint_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
}

bool c_linear_interpolation_inpaint_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !mask.empty() && mask.type() == CV_8UC1 ) {
    linear_interpolation_inpaint(image.getMat(), mask, image);
  }
  return true;
}

