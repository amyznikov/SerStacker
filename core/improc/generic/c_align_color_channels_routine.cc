/*
 * c_align_color_channels_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_align_color_channels_routine.h"
#include <core/proc/reduce_channels.h>

void c_align_color_channels_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  base::getcontrols(ctls, ctx);

  ctlbind(ctls, "reference channel", ctx(&this_class::_reference_channel), "");
  ctlbind(ctls, "enable threshold", ctx(&this_class::_enable_threshold), "");
  ctlbind(ctls, "threshold", ctx(&this_class::_threshold), "");

  ctlbind_expandable_group(ctls, "algorithm", "algorithm parameters");
    c_align_color_channels::getcontrols(ctls, ctx(&this_class::_algorithm));
  ctlbind_end_group(ctls);
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
  if ( !_enable_threshold ) {
    return _algorithm.align(_reference_channel, image.getMat(),
        image, mask.getMat(), mask);
  }

  cv::Mat smask;

  cv::compare(image, _threshold, smask, cv::CMP_GE);
  if ( smask.channels() > 1 ) {
    reduce_color_channels(smask, cv::REDUCE_MAX);
  }
  if ( !mask.empty() ) {
    cv::bitwise_and(mask, smask, smask);
  }

  return _algorithm.align(_reference_channel, image, image, smask);
}

