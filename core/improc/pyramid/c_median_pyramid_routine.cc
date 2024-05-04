/*
 * c_median_pyramid_routine.cc
 *
 *  Created on: Sep 10, 2023
 *      Author: amyznikov
 */

#include "c_median_pyramid_routine.h"
// #include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_median_pyramid_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_median_pyramid_routine::DisplayMedianBlur, "MedianBlur", "Display MedianBlur" },
      { c_median_pyramid_routine::DisplayMedianHat, "MedianHat", "Display MedianHat" },
      { c_median_pyramid_routine::DisplayScaledImage, "ScaledImage", "Display ScaledImage" },
      { c_median_pyramid_routine::DisplayMedianBlur },
  };

  return members;
}


void c_median_pyramid_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, display_type, "Specify display type");
  BIND_PCTRL(ctls, display_level, "Specify display pyramid layer");
  BIND_PCTRL(ctls, ksize, "Specify Median blur kernel size");
  BIND_PCTRL(ctls, median_iterations, "Specify Median iterations");
}

bool c_median_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, display_type);
    SERIALIZE_PROPERTY(settings, save, *this, display_level);
    SERIALIZE_PROPERTY(settings, save, *this, ksize);
    SERIALIZE_PROPERTY(settings, save, *this, median_iterations);
    return true;
  }
  return false;
}

bool c_median_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  build_median_pyramid(image, ksize_, median_iterations_,
      scaled_images_,
      median_blurs_,
      median_hats_);

  if( median_blurs_.empty() || median_hats_.size() != median_blurs_.size() || scaled_images_.size() != median_blurs_.size() ) {
    CF_ERROR("build_median_pyramid() fails");
    return false;
  }

  const int display_level =
      std::max(0, std::min(display_level_,
          (int) median_blurs_.size() - 1));

  switch (display_type_) {
    case DisplayMedianBlur:
      median_blurs_[display_level].copyTo(image);
      break;
    case DisplayMedianHat:
      median_hats_[display_level].copyTo(image);
      break;
    case DisplayScaledImage:
      scaled_images_[display_level].copyTo(image);
      break;
  }

  if ( mask.needed() && !mask.empty() ) {
    mask.release();
  }

  return true;
}
