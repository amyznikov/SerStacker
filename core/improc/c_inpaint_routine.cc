/*
 * c_inpaint_routine.cc
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#include "c_inpaint_routine.h"
#include <core/proc/inpaint.h>


c_inpaint_routine::c_class_factory c_inpaint_routine::class_factory;

c_inpaint_routine::c_inpaint_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_inpaint_routine::ptr c_inpaint_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_inpaint_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  average_pyramid_inpaint(image.getMatRef(), mask,
      image.getMatRef());

  return true;
}


bool c_inpaint_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  return true;
}

bool c_inpaint_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  return true;
}
