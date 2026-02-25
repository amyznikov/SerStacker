/*
 * c_clear_globals_routine.cc
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#include "c_clear_globals_routine.h"

void c_clear_globals_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
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
