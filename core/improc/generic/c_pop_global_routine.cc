/*
 * c_pop_global_routine.cc
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#include "c_pop_global_routine.h"

void c_pop_global_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, artifact_name, "Name for this image to save");
}

bool c_pop_global_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, artifact_name);
    return true;
  }
  return false;
}

bool c_pop_global_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return base::get_global(artifact_name_, image, mask);
}
