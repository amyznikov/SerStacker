/*
 * c_color_balance_routine.cc
 *
 *  Created on: Sep 28, 2023
 *      Author: amyznikov
 */

#include "c_color_balance_routine.h"

void c_color_balance_routine::set_gscale(double v)
{
  gscale_ = v;
}

double c_color_balance_routine::gscale() const
{
  return gscale_;
}

void c_color_balance_routine::set_alpha(double v)
{
  alpha_ = v;
}

double c_color_balance_routine::alpha() const
{
  return alpha_;
}

void c_color_balance_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_DOUBLE_SLIDER_CTRL(ctls, alpha, -1, +1, 1e-4, "");
  ADD_IMAGE_PROCESSOR_DOUBLE_SLIDER_CTRL(ctls, gscale, 0, +2, 1e-4, "");
}

bool c_color_balance_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, alpha);
    SERIALIZE_PROPERTY(settings, save, *this, gscale);
    return true;
  }
  return false;
}

bool c_color_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() && image.channels() == 3 ) {

    const double v =
        alpha_ > 0 ? alpha_ * alpha_ :
            -alpha_ * alpha_;

    const double b = (1 + v );
    const double g = gscale_;
    const double r = (1 - v);

    cv::transform(image.getMat(), image,
        cv::Matx33f(
            b, 0, 0,
            0, g, 0,
            0, 0, r
            ));
  }

  return true;
}
