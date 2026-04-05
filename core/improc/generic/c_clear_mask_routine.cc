/*
 * c_clear_mask_routine.cc
 *
 *  Created on: Apr 4, 2026
 *      Author: amyznikov
 */

#include "c_clear_mask_routine.h"

void c_clear_mask_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
}

bool c_clear_mask_routine::serialize(c_config_setting settings, bool save)
{
  if ( base::serialize(settings, save)) {
    return true;
  }
  return false;
}

bool c_clear_mask_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  mask.release();
  return true;
}


