/*
 * c_crop_image_routine.cc
 *
 *  Created on: Aug 31, 2023
 *      Author: amyznikov
 */

#include "c_crop_image_routine.h"
#include <core/ssprintf.h>

void c_crop_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, CTL_CONTEXT(ctx, opts));
}

bool c_crop_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    serialize_base_roi_selection_options(settings, save, opts);
    return true;
  }
  return false;
}

bool c_crop_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return select_image_roi(opts, image, mask, image, mask);
}
