/*
 * c_flip_image_routine.cc
 *
 *  Created on: Nov 22, 2023
 *      Author: amyznikov
 */

#include "c_flip_image_routine.h"

void c_flip_image_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, hflip, "flip around the Y-axis");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, vflip, "flip around the X-axis");
}

bool c_flip_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, hflip);
    SERIALIZE_PROPERTY(settings, save, *this, vflip);
    return true;
  }
  return false;
}

bool c_flip_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() && (hflip_ || vflip_) ) {

    int flipCode;

    if( hflip_ && vflip_ ) {
      flipCode = -1;
    }
    else if( hflip_ ) {
      flipCode = +1;
    }
    else {
      flipCode = 0;
    }

    cv::flip(image.getMat(), image,
        flipCode);

    if( !mask.empty() ) {

      cv::flip(mask.getMat(), mask,
          flipCode);
    }
  }

  return true;
}

