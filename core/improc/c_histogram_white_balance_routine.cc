/*
 * c_histogram_white_balance_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_histogram_white_balance_routine.h"
#include <core/proc/autoclip.h>
#include <core/proc/reduce_channels.h>

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

void c_histogram_white_balance_routine::set_threshold(double v)
{
  threshold_ = v;
}

double c_histogram_white_balance_routine::threshold() const
{
  return threshold_;
}

void c_histogram_white_balance_routine::set_enable_threshold(bool v)
{
  enable_threshold_ = v;
}

bool c_histogram_white_balance_routine::enable_threshold() const
{
  return enable_threshold_;
}


bool c_histogram_white_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.channels() < 2 ) {
    return true;
  }

  cv::Mat objmask;

  if ( !enable_threshold_ ) {
    objmask = mask.getMat();
  }
  else {
    cv::compare(image, threshold_, objmask, cv::CMP_GE);

    if ( objmask.channels() > 1 ) {
      reduce_color_channels(objmask, cv::REDUCE_MIN);
    }

    if ( !mask.empty() ) {
      cv::bitwise_and(mask, objmask, objmask);
    }
  }

  return histogram_white_balance(image.getMatRef(),
      objmask,
      image.getMatRef(),
      lclip_,
      hclip_);

}


bool c_histogram_white_balance_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("enable_threshold", &enable_threshold_);
  settings.get("threshold", &threshold_);
  settings.get("lclip", &lclip_);
  settings.get("hclip", &hclip_);

  return true;
}

bool c_histogram_white_balance_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("enable_threshold", enable_threshold_);
  settings.set("threshold", threshold_);
  settings.set("lclip", lclip_);
  settings.set("hclip", hclip_);

  return true;
}
