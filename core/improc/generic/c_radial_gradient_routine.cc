/*
 * c_radial_gradient_routine.cc
 *
 *  Created on: Sep 12, 2023
 *      Author: amyznikov
 */

#include "c_radial_gradient_routine.h"
#include  <core/proc/project_to_radius_vector.h>

template<>
const c_enum_member* members_of<c_radial_gradient_routine::OutputType>()
{
  static const c_enum_member members[] = {
      { c_radial_gradient_routine::OutputRadialGradient, "RadialGradient", "Output Radial Gradient" },
      { c_radial_gradient_routine::OutputTangentialGradient, "TangentialGradient", "Output Tangential Gradient" },
      { c_radial_gradient_routine::OutputRadialGradient },
  };

  return members;
}

void c_radial_gradient_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "reference_point", ctx(&this_class::_referencePoint), "Reference point location X,Y [px]");
   ctlbind(ctls, "image center", ctx(&this_class::_imageCenter), "Image center as reference point");
   ctlbind(ctls, "kradius", ctx(&this_class::_kradius), "kernel radius in pixels");
   ctlbind(ctls, "scale", ctx(&this_class::_scale), "Optional value added to the filtered pixels before storing them in dst.");
   ctlbind(ctls, "delta", ctx(&this_class::_delta), "Optional multiplier to differentiate kernel.");
   ctlbind(ctls, "magnitude", ctx(&this_class::_magnitude), "Output gradient magnitude");
   ctlbind(ctls, "squared", ctx(&this_class::_squared), "Square output");
   ctlbind(ctls, "erode_mask", ctx(&this_class::_erode_mask), "Update image mask");
   ctlbind(ctls, "output_type", ctx(&this_class::_output_type), "Output Display");
}

bool c_radial_gradient_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _output_type);
    SERIALIZE_OPTION(settings, save, *this, _referencePoint);
    SERIALIZE_OPTION(settings, save, *this, _kradius);
    SERIALIZE_OPTION(settings, save, *this, _scale);
    SERIALIZE_OPTION(settings, save, *this, _delta);
    SERIALIZE_OPTION(settings, save, *this, _magnitude);
    SERIALIZE_OPTION(settings, save, *this, _squared);
    SERIALIZE_OPTION(settings, save, *this, _erode_mask);
    SERIALIZE_OPTION(settings, save, *this, _imageCenter);

    return true;
  }
  return false;
}

bool c_radial_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat gx, gy, g;

  if ( !compute_gradient(image, gx, 1, 0, _kradius, _scale, _delta) ) {
    CF_ERROR("compute_gradient(x) fails");
    return false;
  }

  if ( !compute_gradient(image, gy, 0, 1, _kradius, _scale, _delta) ) {
    CF_ERROR("compute_gradient(x) fails");
    return false;
  }

  const cv::Point2f rp =
      _imageCenter ? cv::Point2f(image.cols() / 2, image.rows() / 2) :
          _referencePoint;

  switch (_output_type) {
    case OutputRadialGradient:
      project_to_radius_vector(rp, gx, gy, g, cv::noArray());
      break;
    case OutputTangentialGradient:
      project_to_radius_vector(rp, gx, gy, cv::noArray(), g);
      break;
  }

  if( _squared ) {
    cv::multiply(g, g, g);
  }
  else if( _magnitude ) {
    cv::absdiff(g, cv::Scalar::all(0), g);
  }

  image.move(g);

  if ( _erode_mask ) {
    if( mask.needed() && !mask.empty() ) {
      const int r = std::max(1, _kradius);
      cv::erode(mask, mask, cv::Mat1b(2 * r + 1, 2 * r + 1, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
      if ( mask.depth() == CV_8U ) {
        image.setTo(0, ~mask.getMat());
      }
    }
  }

  return true;
}
