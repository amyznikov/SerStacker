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

void c_resize_pyramid_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, scale_factor, "Scale factor");
  BIND_PCTRL(ctls, level, "");
  BIND_PCTRL(ctls, interpolation, "interpolation method, see cv::InterpolationFlags");
  BIND_PCTRL(ctls, display_type, "");
}

bool c_resize_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, scale_factor);
    SERIALIZE_PROPERTY(settings, save, *this, level);
    SERIALIZE_PROPERTY(settings, save, *this, interpolation);
    SERIALIZE_PROPERTY(settings, save, *this, display_type);
    return true;
  }
  return false;
}

bool c_resize_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( level_ > 0 ) {

    double f = 1;
    for ( int i = 0; i < level_; ++i ) {
      f *= scale_factor_;
    }

    if ( display_type_ == DisplayImage ) {

      cv::resize(image.getMat(), image, cv::Size(),
          f, f, interpolation_);
    }
    else {
      cv::Mat tmp;

      cv::resize(image.getMat(), tmp, cv::Size(),
          f, f, interpolation_);

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
