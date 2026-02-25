/*
 * c_resize_image_routine.cc
 *
 *  Created on: May 3, 2024
 *      Author: amyznikov
 */

#include "c_resize_image_routine.h"

void c_resize_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "dstize", ctx(&this_class::_dstize), "output image size.\nIf it equals zero it is computed from fx and fy");
   ctlbind(ctls, "fx", ctx(&this_class::_fx), "scale factor along the horizontal axis;\nIf it equals 0, it is set ftom dstize.width");
   ctlbind(ctls, "fy", ctx(&this_class::_fy), "scale factor along the vertical axis;\nIf it equals 0, it is set ftom dstize.height");
   ctlbind(ctls, "interpolation", ctx(&this_class::_interpolation), "interpolation method, see cv::InterpolationFlags");
   ctlbind(ctls, "mask_interpolation", ctx(&this_class::_mask_interpolation), "mask interpolation method for cv::resize(), see cv::InterpolationFlags");
   ctlbind_spinbox(ctls, "mask_threshold", ctx(&this_class::_mask_threshold), 0, 255, 1, "mask threshold for cv::compare() ");
}

bool c_resize_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _dstize);
    SERIALIZE_OPTION(settings, save, *this, _fx);
    SERIALIZE_OPTION(settings, save, *this, _fy);
    SERIALIZE_OPTION(settings, save, *this, _interpolation);
    SERIALIZE_OPTION(settings, save, *this, _mask_interpolation);
    SERIALIZE_OPTION(settings, save, *this, _mask_threshold);
    return true;
  }
  return false;
}

bool c_resize_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::resize(image.getMat(), image, _dstize, _fx, _fy, _interpolation);

  if( mask.needed() && !mask.empty() ) {
    cv::resize(mask.getMat(), mask, image.size(), 0, 0, _mask_interpolation);
    cv::compare(mask.getMat(), cv::Scalar::all(_mask_threshold), mask, cv::CMP_GE);
  }

  return true;
}

