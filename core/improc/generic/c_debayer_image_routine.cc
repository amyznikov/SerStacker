/*
 * c_debayer_image_routine.cc
 *
 *  Created on: May 26, 2025
 *      Author: amyznikov
 */

#include "c_debayer_image_routine.h"
#include <core/debug.h>

void c_debayer_image_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, colorid, "Select source image colorID ");
  BIND_PCTRL(ctls, method, "Select algorithm used for debayer");
}

bool c_debayer_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, colorid);
    SERIALIZE_PROPERTY(settings, save, *this, method);
    return true;
  }
  return false;
}

bool c_debayer_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !debayer(image.getMat(), image, _colorid, _method) ) {
    CF_ERROR("debayer() fails");
    return false;
  }

  if ( !mask.empty() && mask.size() != image.size() ) {
    cv::resize(mask.getMat(), mask, image.size(), 0, 0, cv::INTER_NEAREST);
  }

  return true;
}

