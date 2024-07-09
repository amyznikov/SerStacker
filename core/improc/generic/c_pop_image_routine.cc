/*
 * c_pop_image_routine.cc
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#include "c_pop_image_routine.h"

void c_pop_image_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, artifact_name, "Name for this image to save");
}

bool c_pop_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, artifact_name);
    return true;
  }
  return false;
}

bool c_pop_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !base::get_artifact(artifact_name_, image, mask) ) {
    CF_ERROR("get_artifact('%s') fails", artifact_name_.c_str());
  }
  return true;
}
