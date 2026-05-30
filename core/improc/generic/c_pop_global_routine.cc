/*
 * c_pop_global_routine.cc
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#include "c_pop_global_routine.h"

void c_pop_global_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "artifact_name", ctx(&this_class::_artifact_name), "Name for image to pop");
  ctlbind(ctls, "pop_image", ctx(&this_class::_pop_image), "");
  ctlbind(ctls, "pop_mask", ctx(&this_class::_pop_mask), "");
}

bool c_pop_global_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _artifact_name);
    SERIALIZE_OPTION(settings, save, *this, _pop_image);
    SERIALIZE_OPTION(settings, save, *this, _pop_mask);
    return true;
  }
  return false;
}

bool c_pop_global_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !base::get_global(_artifact_name, _pop_image ? image : cv::noArray(), _pop_mask ? mask : cv::noArray()) ) {
    CF_ERROR("get_artifact('%s') fails", _artifact_name.c_str());
    return false;
  }
  return true;
}
