/*
 * c_rotate_image_routine.cc
 *
 *  Created on: Jun 10, 2022
 *      Author: amyznikov
 */

#include "c_rotate_image_routine.h"

c_rotate_image_routine::c_class_factory c_rotate_image_routine::class_factory;


c_rotate_image_routine::c_rotate_image_routine(bool enabled) :
    base(&class_factory, enabled)

{
}


c_rotate_image_routine::ptr c_rotate_image_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}


void c_rotate_image_routine::set_rotation_angle(double degrees)
{
  rotation_angle_ = degrees;
}

double c_rotate_image_routine::rotation_angle() const
{
  return rotation_angle_;
}


bool c_rotate_image_routine::deserialize(c_config_setting settings)
{
  return true;
}

bool c_rotate_image_routine::serialize(c_config_setting settings) const
{
  return true;
}

bool c_rotate_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( rotation_angle_ != 0  ) {

    if ( rotation_angle_ == 90 || rotation_angle_ == -270  ) {
      cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
      if ( mask.needed() ) {
        cv::rotate(mask, mask, cv::ROTATE_90_CLOCKWISE);
      }
    }
    else if ( rotation_angle_ == -90 || rotation_angle_ == 270 ) {
      cv::rotate(image.getMat(), image, cv::ROTATE_90_COUNTERCLOCKWISE);
      if ( mask.needed() ) {
        cv::rotate(mask, mask, cv::ROTATE_90_COUNTERCLOCKWISE);
      }
    }
    else if ( rotation_angle_ == 180 || rotation_angle_ == -180 ) {
      cv::rotate(image.getMat(), image, cv::ROTATE_180);
      if ( mask.needed() ) {
        cv::rotate(mask, mask, cv::ROTATE_180);
      }
    }
  }

  return true;
}
