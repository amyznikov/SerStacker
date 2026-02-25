/*
 * c_ridgeness_routine.cc
 *
 *  Created on: May 1, 2024
 *      Author: amyznikov
 */

#include "c_ridgeness_routine.h"

void c_ridgeness_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
}

bool c_ridgeness_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    // SERIALIZE_PROPERTY(settings, save, *this, output_type);
    return true;
  }
  return false;
}

bool c_ridgeness_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask )
{
  ridgeness(image.getMat(), image );
  return true;
}


