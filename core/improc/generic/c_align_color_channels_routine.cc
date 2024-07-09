/*
 * c_align_color_channels_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_align_color_channels_routine.h"
#include <core/proc/reduce_channels.h>



void c_align_color_channels_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, method, "");
  BIND_PCTRL(ctls, reference_channel, "");
  BIND_PCTRL(ctls, motion_type, "");
  BIND_PCTRL(ctls, interpolation, "");
  BIND_PCTRL(ctls, border_mode, "");
  BIND_PCTRL(ctls, border_value, "");
  BIND_PCTRL(ctls, eps, "");
  BIND_PCTRL(ctls, max_iterations, "");
  BIND_PCTRL(ctls, max_level, "");
  BIND_PCTRL(ctls, normalization_level, "");
  BIND_PCTRL(ctls, normalization_eps, "");
  BIND_PCTRL(ctls, smooth_sigma, "");
  BIND_PCTRL(ctls, update_step_scale, "");
  BIND_PCTRL(ctls, enable_threshold, "");
  BIND_PCTRL(ctls, threshold, "");
}

bool c_align_color_channels_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, method);
    SERIALIZE_PROPERTY(settings, save, *this, reference_channel);
    SERIALIZE_PROPERTY(settings, save, *this, motion_type);
    SERIALIZE_PROPERTY(settings, save, *this, interpolation);
    SERIALIZE_PROPERTY(settings, save, *this, border_mode);
    SERIALIZE_PROPERTY(settings, save, *this, border_value);
    SERIALIZE_PROPERTY(settings, save, *this, smooth_sigma);
    SERIALIZE_PROPERTY(settings, save, *this, eps);
    SERIALIZE_PROPERTY(settings, save, *this, max_iterations);
    SERIALIZE_PROPERTY(settings, save, *this, max_level);
    SERIALIZE_PROPERTY(settings, save, *this, update_step_scale);
    SERIALIZE_PROPERTY(settings, save, *this, enable_threshold);
    SERIALIZE_PROPERTY(settings, save, *this, threshold);
    SERIALIZE_PROPERTY(settings, save, *this, normalization_level);
    SERIALIZE_PROPERTY(settings, save, *this, normalization_eps);
    return true;
  }
  return false;
}

bool c_align_color_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !enable_threshold_ ) {
    return algorithm_.align(reference_channel_, image.getMat(),
        image, mask.getMat(), mask);
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

