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
const c_enum_member* members_of<c_gradient_routine::OutputType>()
{
  static constexpr c_enum_member members[] = {
      { c_gradient_routine::OutputGradient, "Gradient", "Gradient" },
      { c_gradient_routine::OutputGradientMagnitude, "Magnitude", "Gradient Magnitude" },
      { c_gradient_routine::OutputGradientPhase, "Phase", "Gradient Phase in range 0..360" },
      { c_gradient_routine::OutputGradientPhase90, "Phase90", "Gradient Phase in range 0..90" },
      { c_gradient_routine::OutputGradientPhase90W, "Phase90W", "Weighted Gradient Phase in range 0..90" },
      { c_gradient_routine::OutputTextureFromGradients, "Texture", "Texture from Gradients" },
      { c_gradient_routine::OutputGradient },
  };

  return members;
}

bool c_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( output_ == OutputGradient ) {

    if ( !compute_gradient(image.getMat(), image, order_x_, order_y_, kradius_, ddepth_, delta_, scale_) ) {
      CF_ERROR("compute_gradient() fails");
      return false;
    }

    if ( squared_ ) {
      cv::multiply(image.getMat(), image.getMat(), image);
    }
  }
  else if( output_ == OutputTextureFromGradients ) {

    cv::Mat gx, gy, g;
    cv::Mat gxx, gyy, gg;

    if ( !compute_gradient(image.getMat(), gx, 1, 0, kradius_, ddepth_, delta_, scale_) ) {
      CF_ERROR("compute_gradient(x) fails");
      return false;
    }
    if ( !compute_gradient(image.getMat(), gy, 0, 1, kradius_, ddepth_, delta_, scale_) ) {
      CF_ERROR("compute_gradient(x) fails");
      return false;
    }
    if ( !compute_gradient(image.getMat(), gxx, 2, 0, kradius_, ddepth_, delta_, scale_) ) {
      CF_ERROR("compute_gradient(x) fails");
      return false;
    }
    if ( !compute_gradient(image.getMat(), gyy, 0, 2, kradius_, ddepth_, delta_, scale_) ) {
      CF_ERROR("compute_gradient(x) fails");
      return false;
    }

    cv::magnitude(gx, gy, g);
    cv::magnitude(gxx, gyy, gg);
    cv::addWeighted(g, 0.25, gg, 0.75, 0, image);
    if ( squared_ ) {
      cv::multiply(image.getMat(), image.getMat(), image);
    }
  }

  else if( output_ == OutputGradientMagnitude ) {

    cv::Mat gx, gy;

    if( order_x_ > 0 && !compute_gradient(image.getMat(), gx, order_x_, 0, kradius_, ddepth_, delta_, scale_) ) {
      CF_ERROR("compute_gradient(gx) fails");
      return false;
    }

    if( order_y_ > 0 && !compute_gradient(image.getMat(), gy, 0, order_y_, kradius_, ddepth_, delta_, scale_) ) {
      CF_ERROR("compute_gradient(gy) fails");
      return false;
    }

    if ( !gx.empty() && !gy.empty() ) {
      cv::magnitude(gx, gy, image);
    }
    else if ( !gx.empty() ) {
      cv::absdiff(gx, cv::Scalar::all(0), image);
    }
    else if ( !gy.empty() ) {
      cv::absdiff(gy, cv::Scalar::all(0), image);
    }
    if ( squared_ ) {
      cv::multiply(image.getMat(), image.getMat(), image);
    }
  }

  else if( output_ == OutputGradientPhase || output_ == OutputGradientPhase90 || output_ == OutputGradientPhase90W ) {

    cv::Mat gx, gy;

    const int order =
        std::max(order_x_, order_y_);

    if( !compute_gradient(image.getMat(), gx, order, 0, kradius_, CV_32F, delta_, scale_) ) {
      CF_ERROR("compute_gradient(gx) fails");
      return false;
    }

    if( !compute_gradient(image.getMat(), gy, 0, order, kradius_, CV_32F, delta_, scale_) ) {
      CF_ERROR("compute_gradient(gy) fails");
      return false;
    }

    switch (output_) {
      case OutputGradientPhase90:
        case OutputGradientPhase90W:
        cv::absdiff(gx, 0, gx);
        cv::absdiff(gy, 0, gy);
        break;
    }

    cv::phase(gx, gy, image, true);

    if (  output_ == OutputGradientPhase90W  ) {
      cv::Mat g;
      cv::magnitude(gx, gy,g);
      if ( squared_ ) {
        cv::multiply(gy,  g,  g);
      }
      cv::multiply(image,  g,  image);
    }
  }


  if( mask.needed() && !mask.empty() ) {
    const int r = std::max(1, kradius_);
    cv::erode(mask, mask, cv::Mat1b(2 * r + 1, 2 * r + 1, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    image.getMatRef().setTo(0, ~mask.getMat());
  }

  return true;
}

