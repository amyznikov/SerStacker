/*
 * c_smap_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_smap_routine.h"

c_smap_routine::c_class_factory c_smap_routine::class_factory;

c_smap_routine::c_smap_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_smap_routine::ptr c_smap_routine::create(bool enabled)
{
  return ptr (new this_class(enabled));
}

c_smap_routine::ptr c_smap_routine::create(double minv, double scale, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_minv(minv);
  obj->set_scale(scale);
  return obj;
}

void c_smap_routine::set_minv(double v)
{
  minv_ = v;
}

double c_smap_routine::minv() const
{
  return minv_;
}

void c_smap_routine::set_scale(double v)
{
  scale_ =  v;
}

double c_smap_routine::scale() const
{
  return scale_;
}

bool c_smap_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat1f smap;
  compute_smap(image, smap, minv_, scale_);
  image.move(smap);
  return true;
}

bool c_smap_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("minv", &minv_);
  settings.get("scale", &scale_);

  return true;
}

bool c_smap_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("minv", minv_);
  settings.set("scale", scale_);

  return true;
}
