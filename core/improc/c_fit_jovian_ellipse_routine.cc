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
const c_enum_member * members_of<c_fit_jovian_ellipse_routine::display_type>()
{
  static constexpr c_enum_member members[] = {
      { c_fit_jovian_ellipse_routine::display_cropped_gray_image, "cropped_gray_image", },
      { c_fit_jovian_ellipse_routine::display_cropped_component_mask, "cropped_component_mask", },
      { c_fit_jovian_ellipse_routine::display_cropped_gradient_image, "cropped_gradient_image", },
      { c_fit_jovian_ellipse_routine::display_cropped_normalized_image, "cropped_normalized_image", },
      { c_fit_jovian_ellipse_routine::display_initial_artificial_ellipse, "initial_artifical_ellipse", },
      { c_fit_jovian_ellipse_routine::display_initial_ellipse_fit, "initial_ellipse_fit", },
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, "final_ellipse_fit", },
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, nullptr, },
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

void c_fit_jovian_ellipse_routine::set_display(display_type v)
{
  display_type_ = v;
}

c_fit_jovian_ellipse_routine::display_type c_fit_jovian_ellipse_routine::display() const
{
  return display_type_;
}

void c_fit_jovian_ellipse_routine::set_hlines(const std::vector<float> & hlines)
{
  detector_.set_hlines(hlines);
}

const std::vector<float> & c_fit_jovian_ellipse_routine::hlines() const
{
  return detector_.hlines();
}

void c_fit_jovian_ellipse_routine::set_normalization_scale(int v)
{
  return detector_.set_normalization_scale(v);
}

int c_fit_jovian_ellipse_routine::normalization_scale() const
{
  return detector_.normalization_scale();
}

void c_fit_jovian_ellipse_routine::set_normalization_blur(double v)
{
  return detector_.set_normalization_blur(v);
}

double c_fit_jovian_ellipse_routine::normalization_blur() const
{
  return detector_.normalization_blur();
}

c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector()
{
  return &detector_;
}

const c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector() const
{
  return &detector_;
}

bool c_fit_jovian_ellipse_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, hlines);
  SAVE_PROPERTY(settings, *this, normalization_scale);
  SAVE_PROPERTY(settings, *this, normalization_blur);
  SAVE_PROPERTY(settings, *this, display);


  return true;
}

bool c_fit_jovian_ellipse_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, hlines);
  LOAD_PROPERTY(settings, this, normalization_scale);
  LOAD_PROPERTY(settings, this, normalization_blur);
  LOAD_PROPERTY(settings, this, display);

  return true;
}

//
///*
// * Five-point approximation to first order image derivative.
// *  <https://en.wikipedia.org/wiki/Numerical_differentiation>
// * */
//static void differentiate(cv::InputArray _src, cv::Mat & gx, cv::Mat & gy, int ddepth)
//{
//  static thread_local const cv::Matx<float, 1, 5> K(
//      (+1.f / 12),
//      (-8.f / 12),
//        0.f,
//      (+8.f / 12),
//      (-1.f / 12));
//
//  if ( ddepth < 0 ) {
//    ddepth = std::max(_src.depth(), CV_32F);
//  }
//
//  const cv::Mat & src = _src.getMat();
//  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//}

bool c_fit_jovian_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
//  c_detect_jovian_ellipse_debug_images debug;
//  cv::RotatedRect ellipse_rect;
//
//  if ( !detect_jovian_ellipse(image, &ellipse_rect, "", &debug, &hlines_) ) {
//    CF_ERROR("detect_jovian_ellipse() fails");
//  }

//  switch (display_type_) {
//  case display_gray_image:
//    debug.gray_image.copyTo(image);
//    break;
//  case display_component_mask:
//    debug.component_mask.copyTo(image);
//    break;
//  case display_gradient_magnitude:
//    debug.cropped_component_image.copyTo(image);
//    break;
//  case display_initial_artifical_ellipse:
//    debug.initial_artifical_ellipse.copyTo(image);
//    break;
//  case display_fitted_artifical_ellipse:
//    debug.fitted_artifical_ellipse.copyTo(image);
//    break;
//  }
//

  detector_.detect_planetary_disk(image, mask);
  switch (display_type_) {
  case display_cropped_gray_image:
    detector_.cropped_gray_image().copyTo(image);
    break;
  case display_cropped_component_mask:
    detector_.uncropped_planetary_disk_mask()(detector_.crop_bounding_box()).copyTo(image);
    break;
  case display_cropped_gradient_image:
    detector_.cropped_gradient_image().copyTo(image);
    break;
  case display_cropped_normalized_image:
    detector_.cropped_normalized_image().copyTo(image);
    break;
  case display_initial_artificial_ellipse:
    detector_.initial_artificial_ellipse().copyTo(image);
    break;
  case display_initial_ellipse_fit:
    detector_.initial_artificial_ellipse_fit().copyTo(image);
    break;
  case display_final_ellipse_fit:
    detector_.cropped_final_ellipse_fit().copyTo(image);
    break;
  }

  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
