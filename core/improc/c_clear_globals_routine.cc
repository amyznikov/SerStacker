/*
 * c_clear_globals_routine.cc
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#include "c_clear_globals_routine.h"

void c_clear_globals_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
//  BIND_PCTRL(ctls, artifact_name, "Name for this image to save");
}

bool c_clear_globals_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    // SERIALIZE_PROPERTY(settings, save, *this, artifact_name);
    return true;
  }
  return false;
}

bool c_clear_globals_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  base::clear_globals();
  return true;
}
