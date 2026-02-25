/*
 * c_push_image_routine.cc
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#include "c_push_image_routine.h"


void c_push_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "artifact_name", ctx(&this_class::_artifact_name), "Name for this image to save");
   ctlbind(ctls, "push_image", ctx(&this_class::_push_image), "");
   ctlbind(ctls, "push_mask", ctx(&this_class::_push_mask), "");
}

bool c_push_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _artifact_name);
    SERIALIZE_OPTION(settings, save, *this, _push_image);
    SERIALIZE_OPTION(settings, save, *this, _push_mask);
    return true;
  }
  return false;
}

bool c_push_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  base::add_artifact(_artifact_name, image, mask);
  return true;
}
