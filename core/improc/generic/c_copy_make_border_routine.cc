/*
 * c_copy_make_border_routine.cc
 *
 *  Created on: Jan 24, 2026
 *      Author: gandriim
 */

#include "c_copy_make_border_routine.h"

void c_copy_make_border_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "borderType", ctx(&this_class::_borderType), "Border mode");
  ctlbind(ctls, "borderValue", ctx(&this_class::_borderValue), "border value for cv::BORDER_CONSTANT");
  ctlbind(ctls, "left", ctx(&this_class::_left), "");
  ctlbind(ctls, "right", ctx(&this_class::_right), "");
  ctlbind(ctls, "top", ctx(&this_class::_top), "");
  ctlbind(ctls, "bottom", ctx(&this_class::_bottom), "");
}

bool c_copy_make_border_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _borderType);
    SERIALIZE_OPTION(settings, save, *this, _borderValue);
    SERIALIZE_OPTION(settings, save, *this, _left);
    SERIALIZE_OPTION(settings, save, *this, _right);
    SERIALIZE_OPTION(settings, save, *this, _top);
    SERIALIZE_OPTION(settings, save, *this, _bottom);
    return true;
  }
  return false;
}

bool c_copy_make_border_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !image.empty() ) {
    cv::copyMakeBorder(image.getMat(), image, _top, _bottom, _left, _right,
      _borderType, _borderValue);
  }

  if ( !mask.empty() ) {
    cv::copyMakeBorder(mask.getMat(), mask, _top, _bottom, _left, _right,
      _borderType, _borderValue);
  }

  return true;
}

