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
      { c_gradient_routine::ComputeMethodScharr, "Scharr", "Use cv::Scharr()" },
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
      { c_gradient_routine::OutputHistogram, "Histogram", "Histogram of Gradient directions" },
      { c_gradient_routine::OutputGradient },
  };

  return members;
}

void c_gradient_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "compute_method", ctx(&this_class::_compute_method), "");
   ctlbind(ctls, "output", ctx(&this_class::_output_type), "");
   ctlbind(ctls, "ddepth", ctx(&this_class::_ddepth), "");
   ctlbind(ctls, "border_type", ctx(&this_class::_border_type), "");
   ctlbind(ctls, "scale", ctx(&this_class::_scale), "");
   ctlbind(ctls, "delta", ctx(&this_class::_delta), "");
   ctlbind(ctls, "squared", ctx(&this_class::_squared), "");
   ctlbind(ctls, "erode_mask", ctx(&this_class::_erode_mask), "");
}

bool c_gradient_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _compute_method);
    SERIALIZE_OPTION(settings, save, *this, _output_type);
    SERIALIZE_OPTION(settings, save, *this, _border_type);
    SERIALIZE_OPTION(settings, save, *this, _scale);
    SERIALIZE_OPTION(settings, save, *this, _ddepth);
    SERIALIZE_OPTION(settings, save, *this, _delta);
    SERIALIZE_OPTION(settings, save, *this, _squared);
    SERIALIZE_OPTION(settings, save, *this, _erode_mask);
    return true;
  }
  return false;
}

bool c_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat gx, gy;

  switch (_compute_method) {
    case ComputeMethodSobel:
      if ( !compute_sobel_gradients(image, gx, gy, _border_type, _scale, _delta) ) {
        CF_ERROR("compute_sobel_gradients() fails");
        return false;
      }
      break;

    case ComputeMethodScharr:
      cv::Scharr(image, gx, CV_32F, 1, 0, _scale, _delta, _border_type);
      cv::Scharr(image, gy, CV_32F, 0, 1, _scale, _delta, _border_type);
      break;

    case ComputeMethodFilter1D:
      if ( !compute_gradient(image, gx, 1, 0, 2, _scale, _delta) ) {
        CF_ERROR("compute_gradient(gx) fails");
        return false;
      }
      if ( !compute_gradient(image, gy, 0, 1, 2, _scale, _delta) ) {
        CF_ERROR("compute_gradient(gx) fails");
        return false;
      }
      break;

      break;
    default:
      CF_ERROR("Invalid compute_method_=%d ('%s') requested ", _compute_method,
          toCString(_compute_method));
      return false;
  }

  switch (_output_type) {
    case OutputGradientX:
      image.move(gx);
      break;

    case OutputGradientY:
      image.move(gy);
      break;

    case OutputGradient: {
      cv::Mat channels[2] = { gx, gy };
      cv::merge(channels, 2, image);
      break;
    }

    case OutputGradientMagnitude:
      cv::magnitude(gx, gy, image);
      if( _squared ) {
        cv::multiply(image.getMat(), image.getMat(), image);
      }
      break;

    case OutputGradientPhase:
      cv::phase(gx, gy, image, true);
      break;

    case OutputHistogram : {
      cv::Mat H;
      compute_histogram_of_gradient_directions(gx, gy, H);
      image.assign(cv::repeat(H.t(), 1, 64));
      mask.release();
      return true;
    }

    default:
      CF_ERROR("Invalid compute_method_=%d ('%s') requested ", _output_type,
          toCString(_output_type));
      return false;
      break;
  }

  if( _erode_mask && mask.needed() && !mask.empty() ) {
    const int r = 2; // std::max(1, kradius_);
    cv::erode(mask, mask, cv::Mat1b(2 * r + 1, 2 * r + 1, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    image.getMatRef().setTo(0, ~mask.getMat());
  }

  return true;
}

