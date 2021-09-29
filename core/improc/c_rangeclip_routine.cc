/*
 * c_rangeclip_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_rangeclip_routine.h"

c_rangeclip_routine::c_class_factory c_rangeclip_routine::class_factory;

c_rangeclip_routine::c_rangeclip_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_rangeclip_routine::ptr c_rangeclip_routine::create(bool enabled)
{
  return ptr (new this_class(enabled));
}

c_rangeclip_routine::ptr c_rangeclip_routine::create(double min, double max, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_min(min);
  obj->set_max(max);
  return obj;
}

void c_rangeclip_routine::set_min(double v)
{
  min_ = v;
}

double c_rangeclip_routine::min() const
{
  return min_;
}

void c_rangeclip_routine::set_max(double v)
{
  max_ = v;
}

double c_rangeclip_routine::max() const
{
  return max_;
}

bool c_rangeclip_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return clip_range(image.getMatRef(), min_, max_/*, mask.getMat()*/);
}


bool c_rangeclip_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("min", &min_);
  settings.get("max", &max_);
  return true;
}

bool c_rangeclip_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("min", min_);
  settings.set("max", max_);
  return true;
}
