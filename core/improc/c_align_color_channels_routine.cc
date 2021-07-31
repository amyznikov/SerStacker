/*
 * c_align_color_channels_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_align_color_channels_routine.h"

c_align_color_channels_routine::c_class_factory c_align_color_channels_routine::class_factory;

c_align_color_channels_routine::c_align_color_channels_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_align_color_channels_routine::ptr c_align_color_channels_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_align_color_channels_routine::set_reference_channel(int v)
{
  reference_channel_ = v;
}

int c_align_color_channels_routine::reference_channel() const
{
  return reference_channel_;
}

bool c_align_color_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return algorithm_.align(reference_channel_, image, image, mask, mask);
}

bool c_align_color_channels_routine::load(c_config_setting settings)
{
  return base::load(settings);
}

bool c_align_color_channels_routine::save(c_config_setting settings) const
{
  return base::save(settings);
}
