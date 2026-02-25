/*
 * c_mean_curvature_blur_routine.cc
 *
 *  Created on: Aug 1, 2022
 *      Author: amyznikov
 */

#include "c_mean_curvature_blur_routine.h"

void c_mean_curvature_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "iterations", ctx(&this_class::_iterations), "Number of iterations");
}

bool c_mean_curvature_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _iterations);
    return true;
  }
  return false;
}

bool c_mean_curvature_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _iterations > 0 ) {
    mean_curvature_blur(image.getMat(), image.getMatRef(), _iterations);
  }
  return true;
}

