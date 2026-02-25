/*
 * c_harris_map_routine.cc
 *
 *  Created on: Jan 25, 2023
 *      Author: amyznikov
 */

#include "c_harris_map_routine.h"

void c_harris_map_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  c_harris_sharpness_measure::getcontrols(ctls, ctx(&this_class::_m));
}

bool c_harris_map_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, k);
    SERIALIZE_PROPERTY(settings, save, *this, dscale);
    SERIALIZE_PROPERTY(settings, save, *this, uscale);
    SERIALIZE_PROPERTY(settings, save, *this, avgchannel);
    return true;
  }
  return false;
}

bool c_harris_map_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  _m.create_map(image.getMat(), image);
  return true;
}

