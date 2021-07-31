/*
 * c_noisemap_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_noisemap_routine.h"

c_noisemap_routine::c_class_factory c_noisemap_routine::class_factory;

c_noisemap_routine::c_noisemap_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_noisemap_routine::ptr c_noisemap_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_noisemap_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  create_noise_map(image.getMatRef(), image.getMatRef(), mask);
  return true;
}

bool c_noisemap_routine::load(c_config_setting settings)
{
  return base::load(settings);
}

bool c_noisemap_routine::save(c_config_setting settings) const
{
  return base::save(settings);
}

