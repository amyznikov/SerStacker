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

c_align_color_channels_routine::ptr c_align_color_channels_routine::create(int ecc_reference_channel,
    ECC_MOTION_TYPE ecc_motion_type, double ecc_eps, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_reference_channel(ecc_reference_channel);

  c_align_color_channels & algorithm = obj->algorithm();
  algorithm.set_motion_type(ecc_motion_type);
  algorithm.set_eps(ecc_eps);

  return obj;
}


void c_align_color_channels_routine::set_reference_channel(int v)
{
  reference_channel_ = v;
}

int c_align_color_channels_routine::reference_channel() const
{
  return reference_channel_;
}

c_align_color_channels & c_align_color_channels_routine::algorithm()
{
  return algorithm_;
}

const c_align_color_channels & c_align_color_channels_routine::algorithm() const
{
  return algorithm_;
}


bool c_align_color_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return algorithm_.align(reference_channel_, image, image, mask, mask);
}

bool c_align_color_channels_routine::deserialize(c_config_setting settings)
{
  return base::deserialize(settings);
}

bool c_align_color_channels_routine::serialize(c_config_setting settings) const
{
  return base::serialize(settings);
}
