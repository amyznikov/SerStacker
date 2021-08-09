/*
 * c_align_color_channels_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_align_color_channels_routine.h"
#include <core/proc/reduce_channels.h>

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

void c_align_color_channels_routine::set_threshold(double v)
{
  threshold_ = v;
}

double c_align_color_channels_routine::threshold() const
{
  return threshold_;
}

void c_align_color_channels_routine::set_enable_threshold(bool v)
{
  enable_threshold_ = v;
}

bool c_align_color_channels_routine::enable_threshold() const
{
  return enable_threshold_;
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

  if ( !enable_threshold_ ) {
    return algorithm_.align(reference_channel_, image, image, mask, mask);
  }

  cv::Mat smask;

  cv::compare(image, threshold_, smask, cv::CMP_GE);
  if ( smask.channels() > 1 ) {
    reduce_color_channels(smask, cv::REDUCE_MAX);
  }
  if ( !mask.empty() ) {
    cv::bitwise_and(mask, smask, smask);
  }

  return algorithm_.align(reference_channel_, image, image, smask);
}

bool c_align_color_channels_routine::deserialize(c_config_setting settings)
{
  settings.get("reference_channel", &reference_channel_);
  settings.get("enable_threshold", &enable_threshold_);
  settings.get("threshold", &threshold_);

  return base::deserialize(settings);
}

bool c_align_color_channels_routine::serialize(c_config_setting settings) const
{
  settings.set("reference_channel", reference_channel_);
  settings.set("enable_threshold", enable_threshold_);
  settings.set("threshold", threshold_);

  return base::serialize(settings);
}
