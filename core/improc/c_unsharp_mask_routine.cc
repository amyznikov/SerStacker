/*
 * c_unsharp_mask_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_unsharp_mask_routine.h"
#include <core/proc/unsharp_mask.h>

c_unsharp_mask_routine::c_class_factory c_unsharp_mask_routine::class_factory;

c_unsharp_mask_routine::c_unsharp_mask_routine(bool enabled)
  : base( &class_factory, enabled)
{
}

c_unsharp_mask_routine::ptr c_unsharp_mask_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_unsharp_mask_routine::ptr c_unsharp_mask_routine::create(double sigma, double alpha, bool enabled) {
  ptr obj(new this_class(enabled));
  obj->set_sigma(sigma);
  obj->set_alpha(alpha);
  return obj;
}

bool c_unsharp_mask_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  unsharp_mask(image, image, sigma_, alpha_, outmin_, outmax_);
  if ( mask.needed() && !mask.empty() ) {
    const int ksize = 2 * std::max(2, (int) (sigma_ * 5)) + 3;
    cv::erode(mask, mask, cv::Mat1b(ksize, ksize, 255), cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
    image.getMatRef().setTo(0, ~mask.getMat());
  }
  return true;
}

void c_unsharp_mask_routine::set_sigma(double v)
{
  sigma_ = v;
}

double c_unsharp_mask_routine::sigma() const
{
  return sigma_;
}

void c_unsharp_mask_routine::set_alpha(double v)
{
  alpha_ = v;
}

double c_unsharp_mask_routine::alpha() const
{
  return alpha_;
}

bool c_unsharp_mask_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("sigma", &sigma_);
  settings.get("alpha", &alpha_);

  return true;
}

bool c_unsharp_mask_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("sigma", sigma_);
  settings.set("alpha", alpha_);

  return true;
}