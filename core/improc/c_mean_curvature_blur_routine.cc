/*
 * c_mean_curvature_blur_routine.cc
 *
 *  Created on: Aug 1, 2022
 *      Author: amyznikov
 */

#include "c_mean_curvature_blur_routine.h"
#include <core/proc/mean_curvature_blur.h>

c_mean_curvature_blur_routine::c_class_factory c_mean_curvature_blur_routine::class_factory;

c_mean_curvature_blur_routine::c_mean_curvature_blur_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_mean_curvature_blur_routine::c_mean_curvature_blur_routine(int iterations, bool enabled) :
    base(&class_factory, enabled),
    iterations_(iterations)
{
}

c_mean_curvature_blur_routine::ptr c_mean_curvature_blur_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_mean_curvature_blur_routine::ptr c_mean_curvature_blur_routine::create(int iterations, bool enabled)
{
  return ptr(new this_class(iterations, enabled));
}

void c_mean_curvature_blur_routine::set_iterations(int v)
{
  iterations_ = v;
}

int c_mean_curvature_blur_routine::iterations() const
{
  return iterations_;
}

bool c_mean_curvature_blur_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, iterations);
  return true;
}

bool c_mean_curvature_blur_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, iterations);

  return true;
}

bool c_mean_curvature_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( iterations_ > 0 ) {
    mean_curvature_blur(image.getMat(), image.getMatRef(), iterations_);
  }
  return true;
}

