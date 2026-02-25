/*
 * c_push_global_routine.cc
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#include "c_push_global_routine.h"


void c_push_global_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "artifact_name", ctx(&this_class::_artifact_name), "Name for this image to save");
}

bool c_push_global_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _artifact_name);
    return true;
  }
  return false;
}

bool c_push_global_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  base::add_global(_artifact_name, image, mask);
  return true;
}
