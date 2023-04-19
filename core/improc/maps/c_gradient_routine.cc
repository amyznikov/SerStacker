/*
 * c_gradient_routine.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "c_gradient_routine.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<c_gradient_routine::OutputType>()
{
  static constexpr c_enum_member members[] = {
    {c_gradient_routine::OutputGradientMagnitude, "Magnitude", " Gradient Magnitude"},
    {c_gradient_routine::OutputGradientX, "GradientX", "GradientX"},
    {c_gradient_routine::OutputGradientY, "GradientY", "GradientY"},
    {c_gradient_routine::OutputGradientMagnitude},
  };

  return members;
}

bool c_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  cv::Mat g;

  switch (output_) {
    case OutputGradientX:
      cv::filter2D(image, g, CV_32F, K, cv::Point(-1, -1), delta_, cv::BORDER_REPLICATE);
      if ( squared_ ) {
        cv::multiply(g, g, g);
      }
      break;

    case OutputGradientY:
      cv::filter2D(image, g, CV_32F, K.t(), cv::Point(-1, -1), delta_, cv::BORDER_REPLICATE);
      if ( squared_ ) {
        cv::multiply(g, g, g);
      }
      break;

    case OutputGradientMagnitude: {
      cv::Mat gx, gy;
      cv::filter2D(image, gx, CV_32F, K, cv::Point(-1, -1), delta_, cv::BORDER_REPLICATE);
      cv::filter2D(image, gy, CV_32F, K.t(), cv::Point(-1, -1), delta_, cv::BORDER_REPLICATE);
      if( squared_ ) {
        cv::add(gx.mul(gx), gy.mul(gy), g);
      }
      else {
        cv::magnitude(gx, gy, g);
      }
      break;
    }
  }

  if ( ddepth_ == CV_32F ) {
    image.move(g);
  }
  else if ( ddepth_ == PIXEL_DEPTH_NO_CHANGE ) {
    g.convertTo(image, image.depth());
  }
  else {
    g.convertTo(image, ddepth_);
  }


  if( mask.needed() && !mask.empty() ) {
    cv::erode(mask, mask, cv::Mat1b(5, 5, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    image.getMatRef().setTo(0, ~mask.getMat());
  }

  return true;
}

