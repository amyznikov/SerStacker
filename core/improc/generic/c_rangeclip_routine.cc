/*
 * c_rangeclip_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_rangeclip_routine.h"

void c_rangeclip_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "min", ctx(&this_class::_min), "");
  ctlbind(ctls, "max", ctx(&this_class::_max), "");
}

bool c_rangeclip_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _min);
    SERIALIZE_OPTION(settings, save, *this, _max);
    return true;
  }
  return false;
}

bool c_rangeclip_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return clip_range(image.getMatRef(), _min, _max/*, mask.getMat()*/);
}

