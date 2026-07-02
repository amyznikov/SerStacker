/*
 * c_gaussian_hpass_routine.cc
 *
 *  Created on: Jul 1, 2026
 *      Author: amyznikov
 */

#include "c_gaussian_hpass_routine.h"

void c_gaussian_hpass_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "sigmax", ctx(&this_class::_sigmax), "");
   ctlbind(ctls, "sigmay", ctx(&this_class::_sigmay), "");
   ctlbind(ctls, "ksizex", ctx(&this_class::_ksizex), "");
   ctlbind(ctls, "ksizey", ctx(&this_class::_ksizey), "");
   ctlbind(ctls, "scale", ctx(&this_class::_scale), "");
   ctlbind(ctls, "delta", ctx(&this_class::_delta), "");
   ctlbind(ctls, "ignore mask", ctx(&this_class::_ignore_mask), "Ignore mask");
   ctlbind(ctls, "border_type", ctx(&this_class::_border_type), "");
   ctlbind(ctls, "border_value", ctx(&this_class::_border_value), "");
}

bool c_gaussian_hpass_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _sigmax);
    SERIALIZE_OPTION(settings, save, *this, _sigmay);
    SERIALIZE_OPTION(settings, save, *this, _ksizex);
    SERIALIZE_OPTION(settings, save, *this, _ksizey);
    SERIALIZE_OPTION(settings, save, *this, _scale);
    SERIALIZE_OPTION(settings, save, *this, _delta);
    SERIALIZE_OPTION(settings, save, *this, _border_type);
    SERIALIZE_OPTION(settings, save, *this, _border_value);
    return true;
  }
  return false;
}

bool c_gaussian_hpass_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  gaussian_hpass_filter(image, mask, image,
      cv::Size2f(_sigmax, _sigmay),
      cv::Size(_ksizex, _ksizey),
      _scale, _delta,
      _border_type, _border_value);

  return true;
}

