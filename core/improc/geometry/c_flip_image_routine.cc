/*
 * c_flip_image_routine.cc
 *
 *  Created on: Nov 22, 2023
 *      Author: amyznikov
 */

#include "c_flip_image_routine.h"

void c_flip_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "hflip", ctx(&this_class::_hflip), "flip around the Y-axis");
   ctlbind(ctls, "vflip", ctx(&this_class::_vflip), "flip around the X-axis");
}

bool c_flip_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _hflip);
    SERIALIZE_OPTION(settings, save, *this, _vflip);
    return true;
  }
  return false;
}

bool c_flip_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() && (_hflip || _vflip) ) {

    int flipCode;

    if( _hflip && _vflip ) {
      flipCode = -1;
    }
    else if( _hflip ) {
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

