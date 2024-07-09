/*
 * c_neighbourhood_average_routine.cc
 *
 *  Created on: Nov 23, 2023
 *      Author: amyznikov
 */

#include "c_neighbourhood_average_routine.h"

void c_neighbourhood_average_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, se_shape, "se_shape");
  BIND_PCTRL(ctls, se_size, "se_size");
  BIND_PCTRL(ctls, anchor, "anchor");

  BIND_PCTRL(ctls, border_type, "border_type");
//  BIND_PCTRL(ctls, border_value, "border_value");

  BIND_PCTRL(ctls, ignore_mask, "ignore_mask");
}

bool c_neighbourhood_average_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_PROPERTY(settings, save, *this, se_shape);

    SERIALIZE_PROPERTY(settings, save, *this, se_size);
    SERIALIZE_PROPERTY(settings, save, *this, anchor);

    SERIALIZE_PROPERTY(settings, save, *this, border_type);
//    SERIALIZE_PROPERTY(settings, save, *this, border_value);

    return true;
  }

  return false;
}

bool c_neighbourhood_average_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {

    cv::Size se_size =
        se_size_;

    cv::Point anchor =
        anchor_;

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

    cv::getStructuringElement(se_shape_, se_size, anchor).
        convertTo(SE, CV_32F);

    SE[anchor.y][anchor.x] = 0;

    cv::divide(SE, cv::sum(SE), SE);

    cv::filter2D(image.getMat(), image, image.depth(),
        SE, anchor, 0,
        border_type_);

  }

  return true;
}
