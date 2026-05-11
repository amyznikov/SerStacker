/*
 * c_average_pyramid_inpaint_routine.cc
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#include "c_average_pyramid_inpaint_routine.h"

void c_average_pyramid_inpaint_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "max_levels", ctx(&this_class::max_levels), "");
}

bool c_average_pyramid_inpaint_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, max_levels);
    return true;
  }
  return false;
}

bool c_average_pyramid_inpaint_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( max_levels > 0 ) {
    cv::Mat outMask;
    average_pyramid_inpaint(image.getMat(), mask, image, outMask, max_levels);
    mask.move(outMask);
  }
  return true;
}

