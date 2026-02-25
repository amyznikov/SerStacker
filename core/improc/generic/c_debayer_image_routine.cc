/*
 * c_debayer_image_routine.cc
 *
 *  Created on: May 26, 2025
 *      Author: amyznikov
 */

#include "c_debayer_image_routine.h"
#include <core/debug.h>

void c_debayer_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "colorid", ctx(&this_class::_colorid), "Source image colorID");
   ctlbind(ctls, "method", ctx(&this_class::_method), "Algorithm used for debayer");
}

bool c_debayer_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _colorid);
    SERIALIZE_OPTION(settings, save, *this, _method);
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

