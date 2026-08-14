/*
 * c_linear_interpolation_inpaint_routine.cc
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#include "c_linear_interpolation_inpaint_routine.h"

void c_linear_interpolation_inpaint_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "inpaintInPlace", CTL_CONTEXT(ctx, _inpaintInPlace));
}

bool c_linear_interpolation_inpaint_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _inpaintInPlace);
    return true;
  }
  return false;
}


bool c_linear_interpolation_inpaint_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  if( !mask.empty() && mask.type() == CV_8UC1 ) {
    if ( _inpaintInPlace ) {
      linear_interpolation_inpaint(image.getMatRef(), mask.getMat());
    }
    else {
      linear_interpolation_inpaint(image, mask, image);
    }
  }
  else {
    CF_ERROR("Bad mask: empty=%d depth=%d channels=%d", mask.empty(), mask.depth(), mask.channels());
  }
  return true;
}

