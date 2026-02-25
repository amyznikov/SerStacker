/*
 * c_absdiff_routine.cc
 *
 *  Created on: Jan 22, 2023
 *      Author: amyznikov
 */

#include "c_absdiff_routine.h"

void c_absdiff_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "value", ctx(&this_class::value), "cv::Scalar");
}

bool c_absdiff_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, value);
    return true;
  }
  return false;
}

bool c_absdiff_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::absdiff(image.getMat(), value, image);
  return true;
}

