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


void c_radial_gradient_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, output_type, "Output type");
  BIND_PCTRL(ctls, reference_point, "Reference point location X,Y [px]");
  BIND_PCTRL(ctls, kradius, "kernel radius in pixels");
  BIND_PCTRL(ctls, delta, "Optional value added to the filtered pixels before storing them in dst.");
  BIND_PCTRL(ctls, scale, "Optional multiplier to differentiate kernel.");
  BIND_PCTRL(ctls, magnitude, "Output gradient magnitude");
  BIND_PCTRL(ctls, squared, "Square output");
  BIND_PCTRL(ctls, erode_mask, "Update image mask if not empty");
}

bool c_radial_gradient_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, output_type);
    SERIALIZE_PROPERTY(settings, save, *this, reference_point);
    SERIALIZE_PROPERTY(settings, save, *this, kradius);
    SERIALIZE_PROPERTY(settings, save, *this, scale);
    SERIALIZE_PROPERTY(settings, save, *this, delta);
    SERIALIZE_PROPERTY(settings, save, *this, magnitude);
    SERIALIZE_PROPERTY(settings, save, *this, squared);
    SERIALIZE_PROPERTY(settings, save, *this, erode_mask);
    return true;
  }
  return false;
}

bool c_radial_gradient_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat gx, gy, g;

  if ( !compute_gradient(image.getMat(), gx, 1, 0, kradius_, CV_32F, delta_, scale_) ) {
    CF_ERROR("compute_gradient(x) fails");
    return false;
  }
  if ( !compute_gradient(image.getMat(), gy, 0, 1, kradius_, CV_32F, delta_, scale_) ) {
    CF_ERROR("compute_gradient(x) fails");
    return false;
  }

  switch (output_type_) {
    case OutputRadialGradient:
      project_to_radius_vector_(gx, gy, reference_point_, &g, nullptr);
      break;
    case OutputTangentialGradient:
      project_to_radius_vector_(gx, gy, reference_point_, nullptr, &g);
      break;
  }

  if( squared_ ) {
    cv::multiply(g, g, g);
  }
  else if( magnitude_ ) {
    cv::absdiff(g, cv::Scalar::all(0), g);
  }

  image.move(g);

  if( mask.needed() && !mask.empty() ) {
    const int r = std::max(1, kradius_);
    cv::erode(mask, mask, cv::Mat1b(2 * r + 1, 2 * r + 1, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    image.getMatRef().setTo(0, ~mask.getMat());
  }

  return true;
}
