/*
 * c_jovian_ellipse_detector.cc
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#include "c_jovian_ellipse_detector.h"
#include <core/proc/autoclip.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/pyrscale.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/pyrscale.h>
#include <core/proc/gradient.h>
#include <core/proc/fitEllipseLM.h>
#include <core/ssprintf.h>
#include <core/settings/opencv_settings.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<JOVIAN_ELLIPSE_DETECTION_METHOD>()
{
  static const c_enum_member members[] = {
    { JOVIAN_ELLIPSE_DETECTION_PCA, "PCA", "" },
    { JOVIAN_ELLIPSE_DETECTION_STENSOR, "STENSOR", "" },
    { JOVIAN_ELLIPSE_DETECTION_PCA }
  };

  return members;
}


static void pnormalize(cv::InputArray _src, cv::OutputArray _dst, int lvl, double eps)
{
  cv::Mat m, s;

  const cv::Mat src = _src.getMat();
  const cv::Size src_size = _src.size();

  pyramid_downscale(src, m, lvl, cv::BORDER_REPLICATE);
  pyramid_downscale(src.mul(src), s, lvl, cv::BORDER_REPLICATE);
  cv::add(s, cv::Scalar::all(eps), s, cv::noArray(), s.depth());
  pyramid_upscale(m, src_size);
  pyramid_upscale(s, src_size);
  cv::subtract(src, m, _dst, cv::noArray(), CV_32F);
  cv::divide(_dst.getMat(), s, _dst);
}


void c_jovian_ellipse_detector::set_options(const c_jovian_ellipse_detector_options & v)
{
  _opts = v;
}

const c_jovian_ellipse_detector_options& c_jovian_ellipse_detector::options() const
{
  return _opts;
}

c_jovian_ellipse_detector_options& c_jovian_ellipse_detector::options()
{
  return _opts;
}

const cv::Mat1b& c_jovian_ellipse_detector::final_planetary_disk_mask() const
{
  return _final_planetary_disk_mask;
}

const cv::Mat1b& c_jovian_ellipse_detector::detected_planetary_disk_mask() const
{
  return _detected_planetary_disk_mask;
}

const cv::Mat1b& c_jovian_ellipse_detector::detected_planetary_disk_edge() const
{
  return _detected_planetary_disk_edge;
}

//const cv::RotatedRect& c_jovian_ellipse_detector::ellipseAMS() const
//{
//  return ellipseAMS_;
//}

const cv::RotatedRect& c_jovian_ellipse_detector::final_planetary_disk_ellipse() const
{
  return _final_planetary_disk_ellipse;
}

const cv::Mat& c_jovian_ellipse_detector::grayscale_image() const
{
  return _grayscale_image;
}

const cv::Mat& c_jovian_ellipse_detector::normalized_image() const
{
  return _normalized_image;
}

const cv::Mat1b & c_jovian_ellipse_detector::gradient_mask() const
{
  return _gradient_mask;
}

const cv::Mat1f & c_jovian_ellipse_detector::g_image() const
{
  return _g;
}

const cv::Mat1f & c_jovian_ellipse_detector::gx_image() const
{
  return _gx;
}

const cv::Mat1f & c_jovian_ellipse_detector::gy_image() const
{
  return _gy;
}

bool c_jovian_ellipse_detector::detect_jovian_ellipse(cv::InputArray _image, cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  cv::RotatedRect ellipse;

  // Convert input image to gray scale
  if( _image.channels() == 1 ) {
    _image.getMat().copyTo(_grayscale_image);
  }
  else {
    cv::cvtColor(_image, _grayscale_image, cv::COLOR_BGR2GRAY);
  }

  // Detect planetary disk and extract pixel mask covering planetary disk shape
  bool fOk =
      simple_planetary_disk_detector(_grayscale_image, _mask,
          _opts.planetary_disk_detector_options,
          nullptr,
          &_detected_component_rect,
          &_detected_planetary_disk_mask);
  if( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }

  extract_channel(_image, _normalized_image, cv::noArray(), cv::noArray(), _opts.gradient_channel);
  if ( _opts.nscale > 0 ) {
    pnormalize(_normalized_image, _normalized_image, _opts.nscale, _opts.neps);
  }

  if( _opts.sigma_noise > 0 ) {
    cv::GaussianBlur(_normalized_image, _normalized_image, cv::Size(0, 0),
        _opts.sigma_noise, _opts.sigma_noise,
        cv::BORDER_REPLICATE);
  }

  const cv::Size size = _detected_component_rect.size();
  const int max_size = std::max(size.width, size.height);
  const int mask_erode_size = std::max(7, 2 * (max_size / 16) + 1);


  // Erode planetary disk mask to avoid edge effects
  cv::erode(_detected_planetary_disk_mask, _gradient_mask,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(mask_erode_size, mask_erode_size)),
      cv::Point(-1, -1),
      1,
      cv::BORDER_CONSTANT,
      cv::Scalar::all(0));


  static float deriv_kernel[] = { +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> Kx = cv::Matx<float, 1, 5>(deriv_kernel);
  static const cv::Matx<float, 5, 1> Ky = cv::Matx<float, 5, 1>(deriv_kernel);

  // Compute derivatives

  cv::filter2D(_normalized_image, _gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(_normalized_image, _gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::magnitude(_gx, _gy, _g);

  const cv::Mat imask = ~_gradient_mask;
  _g.setTo(0, imask);
  if( _opts.gweighted ) {
    cv::multiply(_gx, _g, _gx);
    cv::multiply(_gy, _g, _gy);
  }
  else {
    _gx.setTo(0, imask);
    _gy.setTo(0, imask);
  }


  // Compute orientation angle
  switch (_opts.method) {
    case JOVIAN_ELLIPSE_DETECTION_PCA:
      ellipse.angle = compute_jovian_orientation_pca();
      break;
    case JOVIAN_ELLIPSE_DETECTION_STENSOR:
      ellipse.angle = compute_jovian_orientation_stensor();
      break;
    default:
      CF_ERROR("APP BUG: Not supported method %d requested", _opts.method);
      return false;
  }

  if( ellipse.angle > 90 ) {
    ellipse.angle -= 180;
  }
  else if( ellipse.angle < -90 ) {
    ellipse.angle += 180;
  }

  CF_DEBUG("\nMETHOD: %s angle = %g deg", toCString(_opts.method), ellipse.angle);

  // Use orientation from PCA or HESSIAN above  to compute precise
  // jovian disk fit based on extracted planetary disk edge

  static constexpr double jovian_axis_ratio =
      0.93512560845968779724;

  std::vector<cv::Point2f> ellipse_edge_points;

  morphological_gradient(_detected_planetary_disk_mask,
      _detected_planetary_disk_edge,
      cv::Mat1b(3, 3, 255),
      cv::BORDER_CONSTANT);

  cv::findNonZero(_detected_planetary_disk_edge,
      ellipse_edge_points);

  fOk =
      fitEllipseLM1(ellipse_edge_points,
          jovian_axis_ratio, // b / a
          ellipse.angle * CV_PI / 180, // radians
          &_final_planetary_disk_ellipse);

  // Add optional offset if requested by user
  _final_planetary_disk_ellipse.center += _opts.offset;

  CF_DEBUG("\n"
      "fitEllipseLM1: width=%g height=%g angle=%g",
      _final_planetary_disk_ellipse.size.width,
      _final_planetary_disk_ellipse.size.height,
      _final_planetary_disk_ellipse.angle);

  // create also filed binary ellipse mask
  _final_planetary_disk_mask.create(_image.size());
  _final_planetary_disk_mask.setTo(0);
  cv::ellipse(_final_planetary_disk_mask, _final_planetary_disk_ellipse, 255, -1, cv::LINE_8);
  cv::erode(_final_planetary_disk_mask, _final_planetary_disk_mask, cv::Mat1b(3, 3, 255));

  const double A = _final_planetary_disk_ellipse.size.width / 2;
  const double B = _final_planetary_disk_ellipse.size.width * jovian_axis_ratio / 2;
  const double C = _final_planetary_disk_ellipse.size.width / 2;

  _center = _final_planetary_disk_ellipse.center;
  _axes = cv::Vec3d(A, B, C);
  _pose = build_ellipsoid_pose(0., _opts.planetary_disk_tilt * CV_PI / 180, _final_planetary_disk_ellipse.angle * CV_PI / 180);

  if( true ) {

    CF_DEBUG("FIT:\n"
        "center = %g;%g\n"
        "axes = %g;%g;%g\n"
        "pose = %g;%g;%g\n",
        _center.x, _center.y,
        _axes(0), _axes(1), _axes(2),
        _pose(0) * 180 / CV_PI,
        _pose(1) * 180 / CV_PI,
        _pose(2) * 180 / CV_PI);
  }

  return true;
}

double c_jovian_ellipse_detector::compute_jovian_orientation_stensor()
{
  // Tensor components:Jxx = gx*gx, Jyy = gy*gy, Jxy = gx*gy
  cv::Mat1f jxx, jyy, jxy;
  cv::multiply(_gx, _gx, jxx);
  cv::multiply(_gy, _gy, jyy);
  cv::multiply(_gx, _gy, jxy);

//  const cv::Mat1b imask = ~_maxcolor_mask;
//  jxx.setTo(0, imask);
//  jyy.setTo(0, imask);
//  jxy.setTo(0, imask);

  const double Jxx = cv::sum(jxx)[0];
  const double Jyy = cv::sum(jyy)[0];
  const double Jxy = cv::sum(jxy)[0];

  // Analytical computation of the angle of least change (direction ALONG the stripes).
  // The formula calculates the angle of the leading eigenvector (normal),
  // so to rotate along the equator, we add 90 degrees.
  double angle = 0.5 * std::atan2(2.0 * Jxy, Jxx - Jyy) * 180.0 / CV_PI + 90.0;
  if( angle > 180.0 ) {
    angle -= 360.0;
  }
  if( angle < -180.0 ) {
    angle += 360.0;
  }

  return angle;
}

double c_jovian_ellipse_detector::compute_jovian_orientation_pca()
{
  const int npts = cv::countNonZero(_gradient_mask);
  cv::Mat1f data_pts(npts, 2);

  int ii = 0;

  for( int y = 0; y < _gradient_mask.rows; ++y ) {
    const float * gxp = _gx[y];
    const float * gyp = _gy[y];
    const uint8_t * mskp = _gradient_mask[y];
    for( int x = 0; x < _gradient_mask.cols; ++x ) {
      if( mskp[x] ) {
        const float gx = gxp[x];
        const float gy = gyp[x];
        data_pts[ii][0] = gx;
        data_pts[ii][1] = gy;
        ++ii;
      }
    }
  }

  cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Select second eigen vector for ellipse orientation
  const cv::Vec2f eigen_vec(pca.eigenvectors.at<float>(1, 0), pca.eigenvectors.at<float>(1, 1));
  const double angle = atan2(eigen_vec[1], eigen_vec[0]) * 180 / CV_PI;

  return angle;
}

bool serialize_base_jovian_ellipse_detector_options(c_config_setting section, bool save,
    c_jovian_ellipse_detector_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, method);
  SERIALIZE_OPTION(section, save, opts, gradient_channel);
  SERIALIZE_OPTION(section, save, opts, sigma_noise);
  SERIALIZE_OPTION(section, save, opts, nscale);
  SERIALIZE_OPTION(section, save, opts, neps);
  SERIALIZE_OPTION(section, save, opts, gweighted);
  SERIALIZE_OPTION(section, save, opts, planetary_disk_tilt);
  SERIALIZE_OPTION(section, save, opts, offset);
  serialize_base_planetary_disk_detector_options(section, save, opts.planetary_disk_detector_options);
  return true;
}
