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
  algorithm_.set_motion_type(ECC_MOTION_TRANSLATION);
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

void c_align_color_channels_routine::set_motion_type(ECC_MOTION_TYPE motion_type)
{
  return algorithm_.set_motion_type(motion_type);
}

ECC_MOTION_TYPE c_align_color_channels_routine::motion_type() const
{
  return algorithm_.motion_type();
}

void c_align_color_channels_routine::set_interpolation(cv::InterpolationFlags flags)
{
  return algorithm_.set_interpolation(flags);
}

cv::InterpolationFlags c_align_color_channels_routine::interpolation() const
{
  return algorithm_.interpolation();
}

void c_align_color_channels_routine::set_eps(double v)
{
  return algorithm_.set_eps(v);
}

double c_align_color_channels_routine::eps() const
{
  return algorithm_.eps();
}

void c_align_color_channels_routine::set_max_iterations(int v)
{
  return algorithm_.set_max_iterations(v);
}

int c_align_color_channels_routine::max_iterations() const
{
  return algorithm_.max_iterations();
}

void c_align_color_channels_routine::set_smooth_sigma(double v)
{
  return algorithm_.set_smooth_sigma(v);
}

double c_align_color_channels_routine::smooth_sigma() const
{
  return algorithm_.smooth_sigma();
}

void c_align_color_channels_routine::set_update_step_scale(double v)
{
  return algorithm_.set_update_step_scale(v);
}

double c_align_color_channels_routine::update_step_scale() const
{
  return algorithm_.update_step_scale();
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
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("reference_channel", &reference_channel_);
  settings.get("enable_threshold", &enable_threshold_);
  settings.get("threshold", &threshold_);

  LOAD_PROPERTY(settings, &algorithm_, motion_type);
  LOAD_PROPERTY(settings, &algorithm_, interpolation);
  LOAD_PROPERTY(settings, &algorithm_, smooth_sigma);
  LOAD_PROPERTY(settings, &algorithm_, eps);
  LOAD_PROPERTY(settings, &algorithm_, max_iterations);
  LOAD_PROPERTY(settings, &algorithm_, update_step_scale);

  return true;
}

bool c_align_color_channels_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("reference_channel", reference_channel_);
  settings.set("enable_threshold", enable_threshold_);
  settings.set("threshold", threshold_);

  SAVE_PROPERTY(settings, algorithm_, motion_type);
  SAVE_PROPERTY(settings, algorithm_, interpolation);
  SAVE_PROPERTY(settings, algorithm_, smooth_sigma);
  SAVE_PROPERTY(settings, algorithm_, eps);
  SAVE_PROPERTY(settings, algorithm_, max_iterations);
  SAVE_PROPERTY(settings, algorithm_, update_step_scale);


  return true;
}
