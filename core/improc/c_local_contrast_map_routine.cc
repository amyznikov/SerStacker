/*
 * c_local_contrast_map_routine.cc
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#include "c_local_contrast_map_routine.h"

c_local_contrast_map_routine::c_class_factory c_local_contrast_map_routine::class_factory;

c_local_contrast_map_routine::c_local_contrast_map_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_local_contrast_map_routine::ptr c_local_contrast_map_routine::create(bool enabled)
{
  return ptr (new this_class(enabled));
}

void c_local_contrast_map_routine::set_eps(double v)
{
  eps_ = v;
}

double c_local_contrast_map_routine::eps() const
{
  return eps_;
}

void c_local_contrast_map_routine::set_dscale(int v)
{
  dscale_ = v;
}

int c_local_contrast_map_routine::dscale() const
{
  return dscale_;
}

void c_local_contrast_map_routine::set_equalize_hist(bool v)
{
  equalize_hist_ = v;
}

bool c_local_contrast_map_routine::equalize_hist() const
{
  return equalize_hist_;
}

bool c_local_contrast_map_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  c_local_contrast_measure::compute_contrast_map(image.getMat(), image, eps_, dscale_, equalize_hist_);
  return true;
}

bool c_local_contrast_map_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("eps", &eps_);
  settings.get("dscale", &dscale_);
  settings.get("equalize_hist", &equalize_hist_);

  return true;
}

bool c_local_contrast_map_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("eps", eps_);
  settings.set("dscale", dscale_);
  settings.set("equalize_hist", equalize_hist_);


  return true;
}
