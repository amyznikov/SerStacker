/*
 * c_median_hat_routine.cc
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#include "c_median_hat_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_median_hat_routine::DisplayType>()
{
  static const c_enum_member members[] =  {
      {c_median_hat_routine::DisplayMedianBlur, "MedianBlur", "MedianBlur"} ,
      {c_median_hat_routine::DisplayMedianHat, "MedianHat", "Difference between source image and its median blur" },
      {c_median_hat_routine::DisplayMedianHat },
  };

  return members;
}

void c_median_hat_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "radius", ctx(&this_class::_radius), "Kernetl radius");
   ctlbind(ctls, "iterations", ctx(&this_class::_iterations), "Number of iterations");
   ctlbind(ctls, "display", ctx(&this_class::_display_type), "Output Image display");
   ctlbind(ctls, "absdiff", ctx(&this_class::_absdiff), "Use cv::absdiff() instead of cv::subtract()");
}

bool c_median_hat_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _radius);
    SERIALIZE_OPTION(settings, save, *this, _iterations);
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, *this, _absdiff);
    return true;
  }
  return false;
}

bool c_median_hat_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int r = image.depth() < CV_32F ? _radius : 2;
  const int ksize = 2 * _radius + 1;

  cv::Mat m;

  for( int i = 0; i < _iterations; ++i ) {

    if( m.empty() ) {
      cv::medianBlur(image, m, ksize);
    }
    else {
      cv::medianBlur(m, m, ksize);
    }
  }

  if ( _display_type == DisplayMedianBlur ) {
    image.move(m);
  }
  else if ( _absdiff ) {
    cv::absdiff(image.getMat(), m, image);
  }
  else {

    int ddepth = image.depth();

    switch (ddepth) {
      case CV_8U:
        ddepth = CV_8S;
        break;
      case CV_16U:
        ddepth = CV_16S;
        break;
    }

    cv::subtract(image.getMat(), m, image,
        cv::noArray(),
        ddepth);
  }

  return true;
}
