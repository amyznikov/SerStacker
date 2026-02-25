/*
 * c_median_pyramid_routine.cc
 *
 *  Created on: Sep 10, 2023
 *      Author: amyznikov
 */

#include "c_median_pyramid_routine.h"

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

void c_median_pyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "display_type",  ctx(&this_class::_display_type), "");
   ctlbind(ctls, "ksize",  ctx(&this_class::_ksize), "");
   ctlbind(ctls, "median_iterations",  ctx(&this_class::_median_iterations), "");
   ctlbind(ctls, "display_level",  ctx(&this_class::_display_level), "");
}

bool c_median_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, *this, _display_level);
    SERIALIZE_OPTION(settings, save, *this, _ksize);
    SERIALIZE_OPTION(settings, save, *this, _median_iterations);
    return true;
  }
  return false;
}

bool c_median_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  build_median_pyramid(image, _ksize, _median_iterations,
      scaled_images_,
      median_blurs_,
      median_hats_);

  if( median_blurs_.empty() || median_hats_.size() != median_blurs_.size() || scaled_images_.size() != median_blurs_.size() ) {
    CF_ERROR("build_median_pyramid() fails");
    return false;
  }

  const int display_level =
      std::max(0, std::min(_display_level,
          (int) median_blurs_.size() - 1));

  switch (_display_type) {
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
