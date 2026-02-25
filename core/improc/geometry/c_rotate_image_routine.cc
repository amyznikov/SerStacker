/*
 * c_rotate_image_routine.cc
 *
 *  Created on: Jun 10, 2022
 *      Author: amyznikov
 */

#include "c_rotate_image_routine.h"

bool c_rotate_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _rotation_angle);
    return true;
  }
  return false;
}

bool c_rotate_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _rotation_angle != 0 ) {

    if( _rotation_angle == 90 || _rotation_angle == -270 ) {
      cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
      if( mask.needed() ) {
        cv::rotate(mask, mask, cv::ROTATE_90_CLOCKWISE);
      }
    }
    else if( _rotation_angle == -90 || _rotation_angle == 270 ) {
      cv::rotate(image.getMat(), image, cv::ROTATE_90_COUNTERCLOCKWISE);
      if( mask.needed() ) {
        cv::rotate(mask, mask, cv::ROTATE_90_COUNTERCLOCKWISE);
      }
    }
    else if( _rotation_angle == 180 || _rotation_angle == -180 ) {
      cv::rotate(image.getMat(), image, cv::ROTATE_180);
      if( mask.needed() ) {
        cv::rotate(mask, mask, cv::ROTATE_180);
      }
    }
  }

  return true;
}

void c_rotate_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "rotation_angle", ctx(&this_class::_rotation_angle),
      "rotation angle in degrees: 0, 90, -90, 180, -180, 270,-270");
}
