/*
 * c_pop_image_routine.cc
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#include "c_pop_image_routine.h"

void c_pop_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "artifact_name", ctx(&this_class::_artifact_name), "Name for image to pop");
}

bool c_pop_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _artifact_name);
    return true;
  }
  return false;
}

bool c_pop_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !base::get_artifact(_artifact_name, image, mask) ) {
    CF_ERROR("get_artifact('%s') fails", _artifact_name.c_str());
  }
  return true;
}
