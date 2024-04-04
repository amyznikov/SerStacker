/*
 * c_rotate_image_routine.h
 *
 *  Created on: Jun 10, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rotate_image_routine_h__
#define __c_rotate_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_rotate_image_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_rotate_image_routine,
       "rotate", "Call <strong>cv::rotate()</strong> on image");

  void set_rotation_angle(double degrees)
  {
    rotation_angle_ = degrees;
  }

  double rotation_angle() const
  {
    return rotation_angle_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, rotation_angle, "rotation angle in degrees");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, rotation_angle);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
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

protected:
  double rotation_angle_ = 0 ;
};

#endif /* __c_rotate_image_routine_h__ */
