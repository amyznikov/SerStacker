/*
 * c_autoclip_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_autoclip_routine.h"
#include <core/proc/autoclip.h>

c_autoclip_routine::c_class_factory c_autoclip_routine::class_factory;

c_autoclip_routine::c_autoclip_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_autoclip_routine::ptr c_autoclip_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_autoclip_routine::ptr c_autoclip_routine::create(double lclip, double hclip, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_lclip(lclip);
  obj->set_hclip(hclip);
  return obj;
}

void c_autoclip_routine::set_lclip(double v)
{
  plo_ = v;
}

double c_autoclip_routine::lclip() const
{
  return plo_;
}

void c_autoclip_routine::set_hclip(double v)
{
  phi_ = v;
}

double c_autoclip_routine::hclip() const
{
  return phi_;
}

bool c_autoclip_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  double omin = 0, omax = 1;

  switch ( image.depth() ) {
  case CV_8U :
    omin = 0, omax = UINT8_MAX;
    break;
  case CV_8S :
    omin = INT8_MIN, omax = INT8_MAX;
    break;
  case CV_16U :
    omin = 0, omax = UINT16_MAX;
    break;
  case CV_16S :
    omin = INT16_MIN, omax = INT16_MAX;
    break;
  case CV_32S :
    omin = INT32_MIN, omax = INT32_MAX;
    break;
    break;
  }

  return autoclip(image.getMatRef(), mask, plo_, phi_, omin, omax);
}

bool c_autoclip_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("plo", &plo_);
  settings.get("phi", &phi_);

  return true;
}

bool c_autoclip_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("plo", plo_);
  settings.set("phi", phi_);

  return true;
}
