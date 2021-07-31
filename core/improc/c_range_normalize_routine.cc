/*
 * c_range_normalize_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_range_normalize_routine.h"

c_range_normalize_routine::c_class_factory c_range_normalize_routine::class_factory;

c_range_normalize_routine::c_range_normalize_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_range_normalize_routine::ptr c_range_normalize_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_range_normalize_routine::ptr c_range_normalize_routine::create(double outmin, double outmax, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_outmin(outmin);
  obj->set_outmax(outmax);
  return obj;
}

void c_range_normalize_routine::set_outmin(double v)
{
  outmin_ = v;
}

double c_range_normalize_routine::outmin() const
{
  return outmin_;
}

void c_range_normalize_routine::set_outmax(double v)
{
  outmax_ = v;
}

double c_range_normalize_routine::outmax() const
{
  return outmax_;
}

bool c_range_normalize_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  //clip_range(image.getMatRef(), min_, max_, mask.getMat());
  return normalize_minmax(image.getMatRef(), image.getMatRef(), outmin_, outmax_, mask, true);
}

bool c_range_normalize_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("outmin", &outmin_);
  settings.get("outmax", &outmax_);

  return true;
}

bool c_range_normalize_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("outmin", outmin_);
  settings.set("outmax", outmax_);

  return true;
}
