/*
 * c_image_processor_routine.cc
 *
 *  Created on: Jul 15, 2022
 *      Author: amyznikov
 */

#include "c_median_blur_routine.h"

void c_median_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "radius", ctx(&this_class::_radius), "Kernel radius");
  ctlbind(ctls, "iterations", ctx(&this_class::_iterations), "Number of iterations");
}

bool c_median_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _radius);
    SERIALIZE_OPTION(settings, save, *this, _iterations);
    return true;
  }
  return false;
}

bool c_median_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int r = image.depth() < CV_32F ? _radius : 2;
  const int ksize = 2 * _radius + 1;

  for( int i = 0; i < _iterations; ++i ) {
    cv::medianBlur(image, image, ksize);
  }

  return true;
}
