/*
 * c_fast_gaussian_blur_routine.cc
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#include "c_fast_gaussian_blur_routine.h"
#include <core/proc/fast_gaussian_blur.h>

void c_fast_gaussian_blur_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, sigma, "Gaussian kernel sigma");
  BIND_PCTRL(ctls, ignore_mask, "Ignore mask");
  BIND_PCTRL(ctls, border_type, "Border mode");
  //BIND_PCTRL(ctls, border_value, "border value for cv::BORDER_CONSTANT");
  BIND_PCTRL(ctls, ddepth, "ddepth");

}

bool c_fast_gaussian_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, sigma);
    SERIALIZE_PROPERTY(settings, save, *this, border_type);
    //SERIALIZE_PROPERTY(settings, save, *this, border_value);
    SERIALIZE_PROPERTY(settings, save, *this, ddepth);
    return true;
  }
  return false;
}

bool c_fast_gaussian_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !_ignore_mask && !mask.empty() ) {
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


