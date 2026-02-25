/*
 * c_resize_pyramid_routine.cc
 *
 *  Created on: May 3, 2024
 *      Author: amyznikov
 */

#include "c_resize_pyramid_routine.h"
#include <core/ssprintf.h>


template<>
const c_enum_member* members_of<c_resize_pyramid_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_resize_pyramid_routine::DisplayImage, "Image" },
      { c_resize_pyramid_routine::DisplayAbsdiff, "AbsDiff" },
      { c_resize_pyramid_routine::DisplayImage, },
  };

  return members;
}

void c_resize_pyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "scale_factor",  ctx(&this_class::_scale_factor), "");
   ctlbind(ctls, "level",  ctx(&this_class::_level), "");
   ctlbind(ctls, "interpolation",  ctx(&this_class::_interpolation), "");
   ctlbind(ctls, "display_type",  ctx(&this_class::_display_type), "");
}

bool c_resize_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _scale_factor);
    SERIALIZE_OPTION(settings, save, *this, _level);
    SERIALIZE_OPTION(settings, save, *this, _interpolation);
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    return true;
  }
  return false;
}

bool c_resize_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _level > 0 ) {

    double f = 1;
    for ( int i = 0; i < _level; ++i ) {
      f *= _scale_factor;
    }

    if ( _display_type == DisplayImage ) {

      cv::resize(image.getMat(), image, cv::Size(),
          f, f, _interpolation);
    }
    else {
      cv::Mat tmp;

      cv::resize(image.getMat(), tmp, cv::Size(),
          f, f, _interpolation);

      cv::resize(tmp, tmp, image.size(),
          0, 0, cv::INTER_CUBIC);

      cv::absdiff(image.getMat(), tmp,
          image);
    }

    if( mask.needed() && !mask.empty() ) {
      cv::resize(mask.getMat(), mask, image.size(), 0, 0, cv::INTER_LINEAR);
      cv::compare(mask.getMat(), cv::Scalar::all(254), mask, cv::CMP_GE);
    }
  }

  return true;
}
