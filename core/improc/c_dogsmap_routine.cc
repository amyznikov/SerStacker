/*
 * c_dogsmap_routine.cc
 *
 *  Created on: Oct 16, 2022
 *      Author: amyznikov
 */

#include "c_dogsmap_routine.h"

c_dogsmap_routine::c_class_factory c_dogsmap_routine::class_factory;

c_dogsmap_routine::c_dogsmap_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_dogsmap_routine::c_dogsmap_routine(double s1, double s2, int scale, double minv, bool enabled)
  : base(&class_factory, enabled),
    s1_(s1),
    s2_(s2),
    minv_(minv),
    scale_(scale)
{

}


c_dogsmap_routine::ptr c_dogsmap_routine::create(bool enabled)
{
  return ptr (new this_class(enabled));
}

c_dogsmap_routine::ptr c_dogsmap_routine::create(double s1, double s2, int scale, double minv, bool enabled)
{
  return ptr(new this_class(s1, s2, scale, minv, enabled));
}

void c_dogsmap_routine::set_sigma1(double v)
{
  s1_ = v;
}

double c_dogsmap_routine::sigma1() const
{
  return s1_;
}

void c_dogsmap_routine::set_sigma2(double v)
{
  s2_ = v;
}

double c_dogsmap_routine::sigma2() const
{
  return s2_;
}

void c_dogsmap_routine::set_minv(double v)
{
  minv_ = v;
}

double c_dogsmap_routine::minv() const
{
  return minv_;
}

void c_dogsmap_routine::set_scale(int v)
{
  scale_ = v;
}

int c_dogsmap_routine::scale() const
{
  return scale_;
}

bool c_dogsmap_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  compute_dogsmap(image, image.getMatRef(), s1_, s2_, scale_, minv_);
  return true;
}

bool c_dogsmap_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("s1", &s1_);
  settings.get("s2", &s2_);
  settings.get("scale", &scale_);
  settings.get("minv", &minv_);

  return true;
}

bool c_dogsmap_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("s1", s1_);
  settings.set("s2", s2_);
  settings.set("scale", scale_);
  settings.set("minv", minv_);

  return true;
}

