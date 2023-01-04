/*
 * c_gaussian_filter_routine.cc
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#include "c_gaussian_filter_routine.h"
#include <core/proc/c_gaussian_filter.h>

c_gaussian_filter_routine::c_class_factory c_gaussian_filter_routine::class_factory;

c_gaussian_filter_routine::c_gaussian_filter_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_gaussian_filter_routine::ptr c_gaussian_filter_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_gaussian_filter_routine::ptr c_gaussian_filter_routine::create(double sigma, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_sigma(sigma);
  return obj;
}

void c_gaussian_filter_routine::set_sigma(double v)
{
  sigma_ = v;
}

double c_gaussian_filter_routine::sigma() const
{
  return sigma_;
}


void c_gaussian_filter_routine::set_ignore_mask(bool v)
{
  ignore_mask_ = v;
}

bool c_gaussian_filter_routine::ignore_mask() const
{
  return ignore_mask_;
}

bool c_gaussian_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( ignore_mask_ || mask.empty() || cv::countNonZero(mask) == mask.size().area() ) {
    c_gaussian_filter(sigma_, sigma_).apply(image.getMat(), cv::noArray(), image, cv::BORDER_REFLECT101);
  }
  else {
    cv::Mat tmp;
    image.getMat().copyTo(tmp);
    tmp.setTo(0, ~mask.getMat());
    c_gaussian_filter(sigma_, sigma_).apply(tmp, mask, image, cv::BORDER_REFLECT101);
  }

  return true;
}

bool c_gaussian_filter_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("sigma", &sigma_);
  settings.get("ignore_mask", &ignore_mask_);

  return true;
}

bool c_gaussian_filter_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("sigma", sigma_);
  settings.set("ignore_mask", ignore_mask_);

  return true;
}
