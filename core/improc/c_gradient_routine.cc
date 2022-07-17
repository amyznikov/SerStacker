/*
 * c_gradient_routine.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "c_gradient_routine.h"


c_gradient_routine::c_class_factory c_gradient_routine::class_factory;

c_gradient_routine::c_gradient_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_gradient_routine::ptr c_gradient_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
        0.f,
      (+8.f / 12),
      (-1.f / 12));

  cv::Mat gx, gy;

  cv::filter2D(image, gx, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(image, gy, CV_32F, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::magnitude(gx, gy, image);

  if ( !mask.empty() ) {
    image.getMatRef().setTo(0, ~mask.getMat());
  }

  return true;
}


bool c_gradient_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  return true;
}

bool c_gradient_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  return true;
}
