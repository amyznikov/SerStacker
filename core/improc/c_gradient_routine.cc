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
const c_enum_member* members_of<c_gradient_routine::ComputeMethod>()
{
  static const c_enum_member members[] = {
      { c_gradient_routine::ComputeMethodFilter1D, "Filter1D", "Filter1D" },
      { c_gradient_routine::ComputeMethodSobel, "Sobel", "Use cv::getDerivKernels()" },
      { c_gradient_routine::ComputeMethodDiagonalGradient, "DiagonalGradient", "DiagonalGradient" },
      { c_gradient_routine::ComputeMethodFilter1D },
  };

  return members;
}

template<>
const c_enum_member* members_of<c_gradient_routine::OutputType>()
{
  static const c_enum_member members[] = {
      { c_gradient_routine::OutputGradient, "Gradient", "Gradient" },
      { c_gradient_routine::OutputGradientX, "GradientX", "Gradient along X direction" },
      { c_gradient_routine::OutputGradientY, "GradientY", "Gradient along Y direction" },
      { c_gradient_routine::OutputGradientMagnitude, "Magnitude", "Gradient Magnitude" },
      { c_gradient_routine::OutputGradientPhase, "Phase", "Gradient Phase in range 0..360" },
      { c_gradient_routine::OutputGradientPhaseW, "PhaseW", "Weighted Gradient Phase" },
      { c_gradient_routine::OutputGradientPhase90, "Phase90", "Gradient Phase in range 0..90" },
      { c_gradient_routine::OutputGradientPhase90W, "Phase90W", "Weighted Gradient Phase in range 0..90" },
//      { c_gradient_routine::OutputTextureFromGradients, "Texture", "Texture from Gradients" },
      { c_gradient_routine::OutputGradient },
  };

  return members;
}

void c_gradient_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, compute_method, "method");
  BIND_PCTRL(ctls, output_type, "Output type");
//  BIND_PCTRL(ctls, order_x, "Order of x derivative");
//  BIND_PCTRL(ctls, order_y, "Order of y derivative");
//  BIND_PCTRL(ctls, kradius, "kernel radius in pixels");
  BIND_PCTRL(ctls, ddepth, "Destination image depth");
  BIND_PCTRL(ctls, delta, "Optional value added to the filtered pixels before storing them in dst.");
  BIND_PCTRL(ctls, scale, "Optional multiplier to differentiate kernel.");
  BIND_PCTRL(ctls, squared, "Square output");
  BIND_PCTRL(ctls, erode_mask, "Update image mask if not empty");
}

bool c_gradient_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, compute_method);
    SERIALIZE_PROPERTY(settings, save, *this, output_type);
//    SERIALIZE_PROPERTY(settings, save, *this, order_x);
//    SERIALIZE_PROPERTY(settings, save, *this, order_y);
//    SERIALIZE_PROPERTY(settings, save, *this, kradius);
    SERIALIZE_PROPERTY(settings, save, *this, scale);
    SERIALIZE_PROPERTY(settings, save, *this, ddepth);
    SERIALIZE_PROPERTY(settings, save, *this, delta);
    SERIALIZE_PROPERTY(settings, save, *this, squared);
    SERIALIZE_PROPERTY(settings, save, *this, erode_mask);
    return true;
  }
  return false;
}

bool c_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat gx, gy;

  switch (compute_method_) {
    case ComputeMethodSobel:
      if ( !compute_sobel_gradients(image, gx, gy, ddepth_, border_type_) ) {
        CF_ERROR("compute_sobel_gradients() fails");
        return false;
      }
      break;

    case ComputeMethodDiagonalGradient:
      if ( !compute_diagonal_gradients(image, gx, gy, ddepth_, border_type_) ) {
        CF_ERROR("compute_diagonal_gradients() fails");
        return false;
      }
      break;

    case ComputeMethodFilter1D:
      if ( !compute_gradient(image, gx, 1, 0, 1, ddepth_, delta_, scale_) ) {
        CF_ERROR("compute_gradient(gx) fails");
        return false;
      }
      if ( !compute_gradient(image, gy, 0, 1, 1, ddepth_, delta_, scale_) ) {
        CF_ERROR("compute_gradient(gx) fails");
        return false;
      }
      break;

      break;
    default:
      CF_ERROR("Invalid compute_method_=%d ('%s') requested ", compute_method_,
          toCString(compute_method_));
      return false;
  }



  switch (output_type_) {
    case OutputGradientX:
      image.move(gx);
      break;

    case OutputGradientY:
      image.move(gy);
      break;

    case OutputGradient: {
      cv::Mat channels[2] = {gx, gy};
      cv::merge(channels, 2, image);
      break;
    }

    case OutputGradientMagnitude:
      cv::magnitude(gx, gy, image);
      if( squared_ ) {
        cv::multiply(image.getMat(), image.getMat(), image);
      }
      break;

    case OutputGradientPhase:
      cv::phase(gx, gy, image, true);
      break;

    case OutputGradientPhaseW: {
      cv::phase(gx, gy, image, true);

      cv::Mat g;
      cv::magnitude(gx, gy, g);
      if ( squared_ ) {
        cv::multiply(g,  g,  g);
      }
      cv::multiply(image,  g,  image);
      break;
    }

    case OutputGradientPhase90:
      cv::absdiff(gx, 0, gx);
      cv::absdiff(gy, 0, gy);
      cv::phase(gx, gy, image, true);
      break;

    case OutputGradientPhase90W: {
      cv::Mat g;

      cv::absdiff(gx, 0, gx);
      cv::absdiff(gy, 0, gy);
      cv::magnitude(gx, gy, g);
      if ( squared_ ) {
        cv::multiply(g,  g,  g);
      }

      cv::phase(gx, gy, image, true);
      cv::multiply(image,  g,  image);
      break;
    }

    default:
      CF_ERROR("Invalid compute_method_=%d ('%s') requested ", output_type_,
          toCString(output_type_));
      return false;
      break;
  }

  if( erode_mask_ && mask.needed() && !mask.empty() ) {
    const int r = 2; // std::max(1, kradius_);
    cv::erode(mask, mask, cv::Mat1b(2 * r + 1, 2 * r + 1, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    image.getMatRef().setTo(0, ~mask.getMat());
  }


//
//  if ( output_type_ == OutputGradient ) {
//
//    if ( !compute_gradient(image.getMat(), image, order_x_, order_y_, kradius_, ddepth_, delta_, scale_) ) {
//      CF_ERROR("compute_gradient() fails");
//      return false;
//    }
//
//    if ( squared_ ) {
//      cv::multiply(image.getMat(), image.getMat(), image);
//    }
//  }
//  else if( output_type_ == OutputTextureFromGradients ) {
//
//    cv::Mat gx, gy, g;
//    cv::Mat gxx, gyy, gg;
//
//    if ( !compute_gradient(image.getMat(), gx, 1, 0, kradius_, ddepth_, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(x) fails");
//      return false;
//    }
//    if ( !compute_gradient(image.getMat(), gy, 0, 1, kradius_, ddepth_, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(x) fails");
//      return false;
//    }
//    if ( !compute_gradient(image.getMat(), gxx, 2, 0, kradius_, ddepth_, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(x) fails");
//      return false;
//    }
//    if ( !compute_gradient(image.getMat(), gyy, 0, 2, kradius_, ddepth_, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(x) fails");
//      return false;
//    }
//
//    cv::magnitude(gx, gy, g);
//    cv::magnitude(gxx, gyy, gg);
//    cv::addWeighted(g, 0.25, gg, 0.75, 0, image);
//    if ( squared_ ) {
//      cv::multiply(image.getMat(), image.getMat(), image);
//    }
//  }
//
//  else if( output_type_ == OutputGradientMagnitude ) {
//
//    cv::Mat gx, gy;
//
//    if( order_x_ > 0 && !compute_gradient(image.getMat(), gx, order_x_, 0, kradius_, ddepth_, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(gx) fails");
//      return false;
//    }
//
//    if( order_y_ > 0 && !compute_gradient(image.getMat(), gy, 0, order_y_, kradius_, ddepth_, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(gy) fails");
//      return false;
//    }
//
//    if ( !gx.empty() && !gy.empty() ) {
//      cv::magnitude(gx, gy, image);
//    }
//    else if ( !gx.empty() ) {
//      cv::absdiff(gx, cv::Scalar::all(0), image);
//    }
//    else if ( !gy.empty() ) {
//      cv::absdiff(gy, cv::Scalar::all(0), image);
//    }
//    if ( squared_ ) {
//      cv::multiply(image.getMat(), image.getMat(), image);
//    }
//  }
//
//  else if( output_type_ == OutputGradientPhase || output_type_ == OutputGradientPhase90 || output_type_ == OutputGradientPhase90W ) {
//
//    cv::Mat gx, gy;
//
//    const int order =
//        std::max(order_x_, order_y_);
//
//    if( !compute_gradient(image.getMat(), gx, order, 0, kradius_, CV_32F, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(gx) fails");
//      return false;
//    }
//
//    if( !compute_gradient(image.getMat(), gy, 0, order, kradius_, CV_32F, delta_, scale_) ) {
//      CF_ERROR("compute_gradient(gy) fails");
//      return false;
//    }
//
//    switch (output_type_) {
//      case OutputGradientPhase90:
//        case OutputGradientPhase90W:
//        cv::absdiff(gx, 0, gx);
//        cv::absdiff(gy, 0, gy);
//        break;
//    }
//
//    cv::phase(gx, gy, image, true);
//
//    if (  output_type_ == OutputGradientPhase90W  ) {
//      cv::Mat g;
//      cv::magnitude(gx, gy,g);
//      if ( squared_ ) {
//        cv::multiply(gy,  g,  g);
//      }
//      cv::multiply(image,  g,  image);
//    }
//  }
//
//
//  if( mask.needed() && !mask.empty() ) {
//    const int r = std::max(1, kradius_);
//    cv::erode(mask, mask, cv::Mat1b(2 * r + 1, 2 * r + 1, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//    image.getMatRef().setTo(0, ~mask.getMat());
//  }

  return true;
}

