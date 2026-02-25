/*
 * c_neighbourhood_average_routine.cc
 *
 *  Created on: Nov 23, 2023
 *      Author: amyznikov
 */

#include "c_neighbourhood_average_routine.h"

void c_neighbourhood_average_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "se_shape", ctx(&this_class::_se_shape), "");
   ctlbind(ctls, "se_size", ctx(&this_class::_se_size), "");
   ctlbind(ctls, "anchor", ctx(&this_class::_anchor), "");
   ctlbind(ctls, "border_type", ctx(&this_class::_border_type), "");
}

bool c_neighbourhood_average_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _se_shape);
    SERIALIZE_OPTION(settings, save, *this, _se_size);
    SERIALIZE_OPTION(settings, save, *this, _anchor);
    SERIALIZE_OPTION(settings, save, *this, _border_type);
//    SERIALIZE_OPTION(settings, save, *this, _border_value);

    return true;
  }

  return false;
}

bool c_neighbourhood_average_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {

    cv::Size se_size = _se_size;
    cv::Point anchor = _anchor;

    if ( se_size.width < 1 ) {
      se_size.width = 1;
    }
    if ( se_size.height < 1 ) {
      se_size.height = 1;
    }

    if( anchor.x < 0 || anchor.x >= se_size.width ) {
      anchor.x = se_size.width / 2;
    }

    if( anchor.y < 0 || anchor.y >= se_size.height ) {
      anchor.y = se_size.height / 2;
    }

    cv::Mat1f SE;

    cv::getStructuringElement(_se_shape, se_size, anchor).convertTo(SE, CV_32F);

    SE[anchor.y][anchor.x] = 0;

    cv::divide(SE, cv::sum(SE), SE);

    cv::filter2D(image.getMat(), image, image.depth(),
        SE, anchor, 0,
        _border_type);

  }

  return true;
}
