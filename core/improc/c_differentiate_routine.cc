/*
 * c_differentiate_routine.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "c_differentiate_routine.h"


c_differentiate_routine::c_class_factory c_differentiate_routine::class_factory;

c_differentiate_routine::c_differentiate_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_differentiate_routine::ptr c_differentiate_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_differentiate_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
        0.f,
      (+8.f / 12),
      (-1.f / 12));

  cv::Mat gx, gy;
  cv::filter2D(image, gx, -1, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(image, gy, -1, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::magnitude(gx, gy, image.getMatRef());
  if ( !mask.empty() ) {
    image.getMatRef().setTo(0, ~mask.getMat());
  }

  return true;
}


bool c_differentiate_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  return true;
}

bool c_differentiate_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  return true;
}
