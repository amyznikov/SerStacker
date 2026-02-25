/*
 * c_transpose_image_routine.cc
 *
 *  Created on: Nov 22, 2023
 *      Author: amyznikov
 */

#include "c_transpose_image_routine.h"


void c_transpose_image_routine::getcontrols(c_control_list & /*ctls*/, const ctlbind_context & /*ctx*/)
{
}

bool c_transpose_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    return true;
  }
  return false;
}

bool c_transpose_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !image.empty() ) {
    cv::transpose(image.getMat(), image);
  }
  if ( !mask.empty() ) {
    cv::transpose(mask.getMat(), mask);
  }

  return true;
}
