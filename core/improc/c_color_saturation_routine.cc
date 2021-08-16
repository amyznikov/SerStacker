/*
 * c_color_saturation_routine.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#include "c_color_saturation_routine.h"
#include <core/proc/color_saturation.h>

c_color_saturation_routine::c_class_factory c_color_saturation_routine::class_factory;

c_color_saturation_routine::c_color_saturation_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_color_saturation_routine::ptr c_color_saturation_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_color_saturation_routine::ptr c_color_saturation_routine::create(double scale, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_scale(scale);
  return obj;
}

void c_color_saturation_routine::set_scale(double v)
{
  scale_ = v;
}

double c_color_saturation_routine::scale() const
{
  return scale_;
}

bool c_color_saturation_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, scale);

  return true;
}

bool c_color_saturation_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, scale);

  return true;
}

bool c_color_saturation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return color_saturation_hls(image.getMatRef(), image.getMatRef(), scale_, mask);
}
