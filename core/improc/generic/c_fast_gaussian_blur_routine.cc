/*
 * c_fast_gaussian_blur_routine.cc
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#include "c_fast_gaussian_blur_routine.h"
#include <core/proc/fast_gaussian_blur.h>

void c_fast_gaussian_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "sigma", ctx(&this_class::_sigma), "Gaussian kernel sigma");
  ctlbind(ctls, "ddepth", ctx(&this_class::_ddepth), "Output ddepth");
  ctlbind(ctls, "border_type", ctx(&this_class::_border_type), "Border mode");
  //ctlbind(ctls, "border_value", ctx(&this_class::_border_value), "Border value for cv::BORDER_CONSTANT");
  ctlbind(ctls, "ignore mask", ctx(&this_class::_ignore_mask), "Ignore mask");
}

bool c_fast_gaussian_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _sigma);
    SERIALIZE_OPTION(settings, save, *this, _border_type);
    //SERIALIZE_OPTION(settings, save, *this, _border_value);
    SERIALIZE_OPTION(settings, save, *this, _ddepth);
    return true;
  }
  return false;
}

bool c_fast_gaussian_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !_ignore_mask && !mask.empty() ) {
    image.setTo(0, ~mask.getMat());
  }

  fast_gaussian_blur(image.getMat(),
      _ignore_mask ? cv::noArray() : mask,
      image,
      _sigma,
      _border_type,
      _ddepth);

  return true;
}

