/*
 * c_average_pyramid_inpaint_routine.cc
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#include "c_average_pyramid_inpaint_routine.h"

void c_average_pyramid_inpaint_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
}

bool c_average_pyramid_inpaint_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  average_pyramid_inpaint(image.getMat(), mask, image);
  return true;
}

