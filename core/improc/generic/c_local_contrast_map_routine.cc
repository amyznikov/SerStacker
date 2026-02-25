/*
 * c_local_contrast_map_routine.cc
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#include "c_local_contrast_map_routine.h"

void c_local_contrast_map_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  c_local_contrast_measure::getcontrols(ctls, ctx(&this_class::_m));
}

bool c_local_contrast_map_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, eps);
    SERIALIZE_PROPERTY(settings, save, *this, dscale);
    SERIALIZE_PROPERTY(settings, save, *this, avgchannel);
    return true;
  }
  return false;
}

bool c_local_contrast_map_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  _m.create_map(image.getMat(), image);
  return true;
}

