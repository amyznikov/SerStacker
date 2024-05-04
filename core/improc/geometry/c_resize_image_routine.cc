/*
 * c_resize_image_routine.cc
 *
 *  Created on: May 3, 2024
 *      Author: amyznikov
 */

#include "c_resize_image_routine.h"

void c_resize_image_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, dstize, "output image size.\nIf it equals zero it is computed from fx and fy");
  BIND_PCTRL(ctls, fx, "scale factor along the horizontal axis;\nIf it equals 0, it is set ftom dstize.width");
  BIND_PCTRL(ctls, fy, "scale factor along the vertical axis;\nIf it equals 0, it is set ftom dstize.height");
  BIND_PCTRL(ctls, interpolation, "interpolation method, see cv::InterpolationFlags");
}

bool c_resize_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, dstize);
    SERIALIZE_PROPERTY(settings, save, *this, fx);
    SERIALIZE_PROPERTY(settings, save, *this, fy);
    SERIALIZE_PROPERTY(settings, save, *this, interpolation);
    return true;
  }
  return false;
}

bool c_resize_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::resize(image.getMat(), image, dstize_, fx_, fy_, interpolation_);

  if( mask.needed() && !mask.empty() ) {
    cv::resize(mask.getMat(), mask, image.size(), 0, 0, cv::INTER_LINEAR);
    cv::compare(mask.getMat(), cv::Scalar::all(254), mask, cv::CMP_GE);
  }

  return true;
}

