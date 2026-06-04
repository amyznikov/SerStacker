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
#include <core/proc/project_to_radius_vector.h>
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

static cv::Rect safeExpandROI(const cv::Rect & rc, const cv::Size & image_size, int expand_amount)
{
  const int left = std::max(0, rc.x - expand_amount);
  const int top = std::max(0, rc.y - expand_amount);
  const int right = std::min(image_size.width, rc.x + rc.width + expand_amount);
  const int bottom = std::min(image_size.height, rc.y + rc.height + expand_amount);
  return cv::Rect(left, top, right-left, bottom-top);
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

// Compute first-order derivatives
static void differentiate(cv::InputArray _src, cv::OutputArray gx, cv::OutputArray gy, double gbsigma = 0)
{
  static float deriv_kernel[] = { +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> Kx = cv::Matx<float, 1, 5>(deriv_kernel);
  static const cv::Matx<float, 5, 1> Ky = cv::Matx<float, 5, 1>(deriv_kernel);

  if( !(gbsigma > 0) ) {
    cv::filter2D(_src, gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(_src, gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }
  else {
    cv::Mat tmp;
    cv::GaussianBlur(_src, tmp, cv::Size(0, 0), gbsigma, gbsigma, cv::BORDER_REPLICATE);
    cv::filter2D(tmp, gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(tmp, gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  }
}

static void extract_points(const cv::Mat1f & image, const cv::Mat1b & mask, const cv::Rect & roi,
    std::vector<cv::Point3f> & pts, bool weighted = true)
{
  for ( int y = roi.y; y < roi.y + roi.height; ++y ) {
    const float * srcp = image[y];
    const uint8_t * mp = mask[y];
    for ( int x = roi.x; x < roi.x + roi.width; ++x ) {
      if ( mp[x] )  {
        pts.emplace_back(x, y, weighted ? srcp[x] : 1.f);
      }
    }
  }
}

void c_jovian_ellipse_detector::set_options(const c_jovian_ellipse_detector_options & v)
{
  _opts = v;
}

const c_jovian_ellipse_detector_options& c_jovian_ellipse_detector::options() const
{
  return _opts;
}

const cv::Mat1b& c_jovian_ellipse_detector::final_planetary_disk_mask() const
{
  return _final_planetary_disk_mask;
}

const cv::Mat1b& c_jovian_ellipse_detector::disk_mask() const
{
  return _disk_mask;
}

const cv::Mat1b& c_jovian_ellipse_detector::disk_edge() const
{
  return _disk_edge;
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

const cv::Mat1f & c_jovian_ellipse_detector::gr_image() const
{
  return _gr;
}

const cv::Mat1f & c_jovian_ellipse_detector::grth_image() const
{
  return _grth;
}

void c_jovian_ellipse_detector::clear()
{
  _grayscale_image.release();
  _normalized_image.release();
  _disk_mask.release();
  _disk_edge.release();
  _gradient_mask.release();
  _gx.release();
  _gy.release();
  _g.release();
  _gr.release();
  _grth.release();
  _final_planetary_disk_mask.release();
}

bool c_jovian_ellipse_detector::detect_planetary_disk_mask(cv::InputArray _input_image, cv::InputArray _input_mask)
{
  // Convert input image to gray scale
  if( _input_image.channels() == 1 ) {
    _input_image.getMat().copyTo(_grayscale_image);
  }
  else {
    cv::cvtColor(_input_image, _grayscale_image, cv::COLOR_BGR2GRAY);
  }

  // Detect planetary disk and extract pixel mask covering planetary disk shape
  bool fOk =
      simple_planetary_disk_detector(_grayscale_image, _input_mask,
          _opts.planetary_disk_detector_options,
          &_detected_component_centroid,
          &_detected_component_roi,
          &_disk_mask);
  if( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }

  const cv::Size size = _detected_component_roi.size();
  const int max_size = std::max(size.width, size.height);
  _skirt_size = std::max(11, 2 * (max_size / 48) + 1);
  _gradient_mask_erode_size = std::max(7, 2 * (max_size / 16) + 1);

  morphological_gradient(_disk_mask, _disk_edge,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_skirt_size, _skirt_size)),
      cv::BORDER_REPLICATE);

  cv::erode(_disk_mask, _gradient_mask,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_gradient_mask_erode_size, _gradient_mask_erode_size)),
      cv::Point(-1, -1),
      1,
      cv::BORDER_CONSTANT,
      cv::Scalar::all(0));

  return true;
}


bool c_jovian_ellipse_detector::detect(cv::InputArray _image, cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  if ( !detect_planetary_disk_mask(_image, _mask) ) {
    CF_ERROR("detect_planetary_disk_mask() fails");
    return false;
  }

  // Prepare gr, grth images for contour detection
  static const cv::Mat1b THSE(5, 5, uint8_t(255));
  extract_channel(_image, _normalized_image, cv::noArray(), cv::noArray(), _opts.gradient_channel);
  differentiate(_normalized_image, _gx, _gy, _opts.sigma_contour);
  project_to_radius_vector(_detected_component_centroid, _gx, _gy, _gr, cv::noArray());
  cv::multiply(_gr, cv::Scalar::all(-1e4), _gr);
  cv::max(_gr, 0.f, _grth);
  cv::morphologyEx(_grth, _grth, cv::MORPH_TOPHAT, THSE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  _gr.setTo(0, ~_disk_edge);
  cv::sqrt(_gr, _gr);

  // Prepare gx, gy, g images for orientation detection
  if ( _opts.nscale > 0 ) {
    pnormalize(_normalized_image, _normalized_image, _opts.nscale, _opts.neps);
  }
  differentiate(_normalized_image, _gx, _gy, _opts.sigma_clouds);
  cv::magnitude(_gx, _gy, _g);

  // Compute orientation angle
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

  double ellipse_angle_deg = 0;

  switch (_opts.method) {
    case JOVIAN_ELLIPSE_DETECTION_PCA:
      ellipse_angle_deg = compute_jovian_orientation_pca();
      break;
    case JOVIAN_ELLIPSE_DETECTION_STENSOR:
      ellipse_angle_deg = compute_jovian_orientation_stensor();
      break;
    default:
      CF_ERROR("APP BUG: Not supported method %d requested", _opts.method);
      return false;
  }
  if( ellipse_angle_deg > 90 ) {
    ellipse_angle_deg -= 180;
  }
  else if( ellipse_angle_deg < -90 ) {
    ellipse_angle_deg += 180;
  }

  CF_DEBUG("\nMETHOD: %s angle = %g deg", toCString(_opts.method), ellipse_angle_deg);

  // Use orientation from PCA or HESSIAN above to improve
  // jovian disk fit based on extracted planetary disk edge

  constexpr double axis_ratio = k_jovian_axis_ratio;
  const double tilt_to_earth = _opts.planetary_disk_tilt * CV_PI / 180;
  const double cos_tilt = cos(tilt_to_earth);
  const double sin_tilt = sin(tilt_to_earth);
  const double fixed_axis_ratio = std::sqrt(axis_ratio * axis_ratio * cos_tilt * cos_tilt + sin_tilt * sin_tilt);

  std::vector<cv::Point3f> ellipse_edge_points;

  extract_points(_grth, _disk_edge,
      safeExpandROI(_detected_component_roi, _grth.size(), _skirt_size),
      ellipse_edge_points,
      _opts.lmweighted);

  fitEllipseLMW(ellipse_edge_points,
      fixed_axis_ratio, // b / a
      ellipse_angle_deg * CV_PI / 180, // radians
      &_final_planetary_disk_ellipse);

  // Add optional offset if requested by user
  _final_planetary_disk_ellipse.center += _opts.offset;

  // Create also filed binary ellipse mask
  draw_ellipse_mask(_final_planetary_disk_mask, _image.size(), _final_planetary_disk_ellipse);

  const double A = _final_planetary_disk_ellipse.size.width / 2;
  const double B = _final_planetary_disk_ellipse.size.width * axis_ratio / 2;
  const double C = _final_planetary_disk_ellipse.size.width / 2;
  _center = _final_planetary_disk_ellipse.center;
  _axes = cv::Vec3d(A, B, C);
  _pose = build_ellipsoid_pose(0., _opts.planetary_disk_tilt * CV_PI / 180, _final_planetary_disk_ellipse.angle * CV_PI / 180);

  if( true ) {
    const std::string s = serialize_ellipsoid_to_string(_center, _axes, _pose * 180 / CV_PI);
    CF_DEBUG("\nfitEllipseLMW:\n" "%s\n", s.c_str());
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
  SERIALIZE_OPTION(section, save, opts, sigma_contour);
  SERIALIZE_OPTION(section, save, opts, sigma_clouds);
  SERIALIZE_OPTION(section, save, opts, nscale);
  SERIALIZE_OPTION(section, save, opts, neps);
  SERIALIZE_OPTION(section, save, opts, gweighted);
  SERIALIZE_OPTION(section, save, opts, lmweighted);
  SERIALIZE_OPTION(section, save, opts, planetary_disk_tilt);
  SERIALIZE_OPTION(section, save, opts, offset);
  serialize_base_planetary_disk_detector_options(section, save, opts.planetary_disk_detector_options);
  return true;
}
