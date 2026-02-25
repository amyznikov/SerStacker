/*
 * c_radial_gradient_routine.cc
 *
 *  Created on: Sep 12, 2023
 *      Author: amyznikov
 */

#include "c_radial_gradient_routine.h"

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

static bool project_to_radius_vector_(const cv::Mat & _gx, const cv::Mat & _gy, const cv::Point2f & rp,
    cv::Mat * _gr, cv::Mat * _gt)
{
  const int rows =
      _gx.rows;

  const int cols =
      _gx.cols;

  const int cn =
      _gx.channels();

  if( _gr ) {
    _gr->create(rows, cols,
        CV_MAKETYPE(CV_32F, cn));
  }

  if( _gt ) {
    _gt->create(rows, cols,
        CV_MAKETYPE(CV_32F, cn));
  }

  const cv::Mat_<float> gx =
      _gx;

  const cv::Mat_<float> gy =
      _gy;

  cv::Mat_<float> gr =
      _gr ? *_gr : cv::Mat_<float>();

  cv::Mat_<float> gt =
      _gt ? *_gt : cv::Mat_<float>();

  for ( int y = 0; y < rows; ++y ) {

    const float * gxp = gx[y];
    const float * gyp = gy[y];
    float * grp = _gr ? gr[y] : nullptr;
    float * gtp = _gt ? gt[y] : nullptr;

    for ( int x = 0; x < cols; ++x ) {

      const float dx = x - rp.x;
      const float dy = y - rp.y;
      const float dr = std::max(2 * FLT_EPSILON, std::sqrt(dx * dx + dy * dy));
      const float ca = dx / dr;
      const float sa = dy / dr;

      for ( int c = 0; c < cn; ++c ) {

        const float & gxv =
            gxp[x * cn + c];

        const float & gyv =
            gyp[x * cn + c];

        if ( grp ) {
          grp[x * cn + c] =
              (+gxv * ca + gyv * sa);
        }

        if ( gtp ) {
          gtp[x * cn + c] =
              (-gyv * ca + gxv * sa);
        }
      }
    }
  }

  return true;
}

void c_radial_gradient_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "reference_point", ctx(&this_class::_reference_point), "Reference point location X,Y [px]");
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
    SERIALIZE_OPTION(settings, save, *this, _reference_point);
    SERIALIZE_OPTION(settings, save, *this, _kradius);
    SERIALIZE_OPTION(settings, save, *this, _scale);
    SERIALIZE_OPTION(settings, save, *this, _delta);
    SERIALIZE_OPTION(settings, save, *this, _magnitude);
    SERIALIZE_OPTION(settings, save, *this, _squared);
    SERIALIZE_OPTION(settings, save, *this, _erode_mask);
    return true;
  }
  return false;
}

bool c_radial_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat gx, gy, g;

  if ( !compute_gradient(image.getMat(), gx, 1, 0, _kradius, CV_32F, _delta, _scale) ) {
    CF_ERROR("compute_gradient(x) fails");
    return false;
  }
  if ( !compute_gradient(image.getMat(), gy, 0, 1, _kradius, CV_32F, _delta, _scale) ) {
    CF_ERROR("compute_gradient(x) fails");
    return false;
  }

  switch (_output_type) {
    case OutputRadialGradient:
      project_to_radius_vector_(gx, gy, _reference_point, &g, nullptr);
      break;
    case OutputTangentialGradient:
      project_to_radius_vector_(gx, gy, _reference_point, nullptr, &g);
      break;
  }

  if( _squared ) {
    cv::multiply(g, g, g);
  }
  else if( _magnitude ) {
    cv::absdiff(g, cv::Scalar::all(0), g);
  }

  image.move(g);

  if( mask.needed() && !mask.empty() ) {
    const int r = std::max(1, _kradius);
    cv::erode(mask, mask, cv::Mat1b(2 * r + 1, 2 * r + 1, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    image.getMatRef().setTo(0, ~mask.getMat());
  }

  return true;
}
