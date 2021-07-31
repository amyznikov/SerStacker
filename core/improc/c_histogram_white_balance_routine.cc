/*
 * c_histogram_white_balance_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_histogram_white_balance_routine.h"
#include <core/proc/autoclip.h>

c_histogram_white_balance_routine::c_class_factory c_histogram_white_balance_routine::class_factory;

c_histogram_white_balance_routine::c_histogram_white_balance_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_histogram_white_balance_routine::ptr c_histogram_white_balance_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_histogram_white_balance_routine::ptr c_histogram_white_balance_routine::create(double lclip, double hclip, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_lclip(lclip);
  obj->set_hclip(hclip);
  return obj;
}

void c_histogram_white_balance_routine::set_lclip(double v)
{
  lclip_ = v;
}

double c_histogram_white_balance_routine::lclip() const
{
  return lclip_;
}

void c_histogram_white_balance_routine::set_hclip(double v)
{
  hclip_ = v;
}

double c_histogram_white_balance_routine::hclip() const
{
  return hclip_;
}

bool c_histogram_white_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return histogram_white_balance(image.getMatRef(),
      mask,
      image.getMatRef(),
      lclip_,
      hclip_);
}


bool c_histogram_white_balance_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  settings.get("lclip", &lclip_);
  settings.get("hclip", &hclip_);

  return true;
}

bool c_histogram_white_balance_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("lclip", lclip_);
  settings.set("hclip", hclip_);

  return true;
}
