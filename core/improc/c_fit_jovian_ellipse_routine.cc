/*
 * c_fit_jovian_ellipse_routine.cc
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#include "c_fit_jovian_ellipse_routine.h"
#include <core/ssprintf.h>

c_fit_jovian_ellipse_routine::c_class_factory c_fit_jovian_ellipse_routine::class_factory;

template<>
const c_enum_member * members_of<c_fit_jovian_ellipse_routine::display_image_type>()
{
  static constexpr c_enum_member members[] = {
      { c_fit_jovian_ellipse_routine::display_gray_image, "gray_image", },
      { c_fit_jovian_ellipse_routine::display_component_mask, "component_mask", },
      { c_fit_jovian_ellipse_routine::display_gradient_magnitude, "gradient", },
      { c_fit_jovian_ellipse_routine::display_initial_artifical_ellipse, "initial_ellipse", },
      { c_fit_jovian_ellipse_routine::display_fitted_artifical_ellipse, "fitted_ellipse", },
      { c_fit_jovian_ellipse_routine::display_fitted_artifical_ellipse, nullptr, },
  };

  return members;
}


c_fit_jovian_ellipse_routine::c_fit_jovian_ellipse_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_fit_jovian_ellipse_routine::ptr c_fit_jovian_ellipse_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_fit_jovian_ellipse_routine::set_display_type(display_image_type v)
{
  display_type_ = v;
}

c_fit_jovian_ellipse_routine::display_image_type c_fit_jovian_ellipse_routine::display_type() const
{
  return display_type_;
}

bool c_fit_jovian_ellipse_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, display_type);

  return true;
}

bool c_fit_jovian_ellipse_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, display_type);

  return true;
}

bool c_fit_jovian_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  c_detect_jovian_ellipse_debug_images debug;
  cv::RotatedRect ellipse_rect;

  if ( !detect_jovian_ellipse(image, &ellipse_rect, "", &debug) ) {
    CF_ERROR("detect_jovian_ellipse() fails");
  }

  if ( mask.needed() ) {
    mask.release();
  }

  switch (display_type_) {
  case display_gray_image:
    debug.gray_image.copyTo(image);
    break;
  case display_component_mask:
    debug.component_mask.copyTo(image);
    break;
  case display_gradient_magnitude:
    debug.gradient_magnitude.copyTo(image);
    break;
  case display_initial_artifical_ellipse:
    debug.initial_artifical_ellipse.copyTo(image);
    break;
  case display_fitted_artifical_ellipse:
    debug.fitted_artifical_ellipse.copyTo(image);
    break;
  }


//  cv::RotatedRect output_ellipse;
//  cv::Rect output_ellipse_boundig_box;
//  cv::Mat output_component_image;
//  cv::Mat output_component_mask;
//  cv::Mat1b output_ellipse_mask;
//
//  extract_jovian_image(image, mask,
//      &output_ellipse,
//      &output_ellipse_boundig_box,
//      &output_component_image,
//      &output_component_mask,
//      &output_ellipse_mask);

  return true;
}
