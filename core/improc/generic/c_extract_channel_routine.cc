/*
 * c_extract_channel_routine.cc
 *
 *  Created on: Jan 13, 2023
 *      Author: amyznikov
 */

#include "c_extract_channel_routine.h"

void c_extract_channel_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "output_channel", ctx(&this_class::_output_channel), "channel index or enum color_channel_type");
  ctlbind(ctls, "output_depth", ctx(&this_class::_output_depth), "output depth");
  ctlbind(ctls, "autoscale", ctx(&this_class::_autoscale), "optional output depth scale");
}

bool c_extract_channel_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _output_channel);
    SERIALIZE_OPTION(settings, save, *this, _output_depth);
    SERIALIZE_OPTION(settings, save, *this, _autoscale);
    return true;
  }
  return false;
}

bool c_extract_channel_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return extract_channel(image.getMat(), image, mask.getMat(), mask,
      _output_channel,
      _output_depth,
      _autoscale);
}

