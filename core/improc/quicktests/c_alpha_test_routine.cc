/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>


template<>
const c_enum_member * members_of<c_alpha_test_routine::MoonProjection>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::ProjectionOrthographic, "Orthographic", "" },
      { c_alpha_test_routine::ProjectionStereographic, "Stereographic", "" },
      { c_alpha_test_routine::ProjectionOrthographic}
  };
  return members;
}

template<>
const c_enum_member * members_of<c_alpha_test_routine::ResizeMode>()
{
  static const c_enum_member members[] = {
      {c_alpha_test_routine::ResizeModeKeep, "KEEP", ""},
      {c_alpha_test_routine::ResizeModeAdjust, "ADJUST", ""},
      {c_alpha_test_routine::ResizeModeCropVisible, "CropVisible", ""},
      {c_alpha_test_routine::ResizeModeCropVisible},
  };

  return members;
}


namespace {

using MoonProjection = c_alpha_test_routine::MoonProjection;


struct c_lunar_birdview_options {
    double lon = 0;           // Crater longitude [deg]
    double lat = 0;           // Crater latitude [deg]
    double l = 0;             // Libration in longitude [deg]
    double b = 0;             // Libration in latitude [deg]
    double camera_rotation = 0;  // Photo orientation angle [deg]
    c_alpha_test_routine::ResizeMode resizeMode;
    int interpolation = cv::INTER_LINEAR;
    int borderMode = cv::BORDER_CONSTANT;
    cv::Scalar borderValue;
};

static void createMoonUnrollMaps(const c_lunar_birdview_options & opts,
    const cv::Size & src_size, const cv::Size & dst_size,
    double R_moon_pixels,
    cv::Mat & map_x,
    cv::Mat & map_y,
    MoonProjection proj_type)
{
  map_x.create(dst_size, CV_32FC1);
  map_y.create(dst_size, CV_32FC1);

  const double cx_src = src_size.width / 2.0;
  const double cy_src = src_size.height / 2.0;
  const double cx_dst = dst_size.width / 2.0;
  const double cy_dst = dst_size.height / 2.0;

  const double lon0 = opts.lon * CV_PI / 180.0;
  const double lat0 = opts.lat * CV_PI / 180.0;
  const double l = opts.l * CV_PI / 180.0;
  const double b = opts.b * CV_PI / 180.0;
  const double cam_rot = opts.camera_rotation * CV_PI / 180.0;

  // Scaling factor for projections so that pixels in the center of the frame are aligned 1:1 with the original
  // For orthographic projection the factor is equal to R_moon_pixels
  // For stereographic projection the factor is equal to 2.0 * R_moon_pixels
  const double scale_factor =
      (proj_type == c_alpha_test_routine::ProjectionOrthographic) ? R_moon_pixels : (2.0 * R_moon_pixels);

  for( int y = 0; y < dst_size.height; ++y ) {
    float * p_map_x = map_x.ptr<float>(y);
    float * p_map_y = map_y.ptr<float>(y);

    for( int x = 0; x < dst_size.width; ++x ) {
      // Coordinates on the target plane relative to the central point of interest
      double dx = x - cx_dst;
      double dy = y - cy_dst;

      double lon_p = 0, lat_p = 0;
      double rho = std::hypot(dx, dy);

      if( rho < 1e-6 ) {
        lon_p = lon0;
        lat_p = lat0;
      }
      else {
        if( proj_type == c_alpha_test_routine::ProjectionOrthographic ) {
          // Inverse orthographic projection relative to the central point (lon0, lat0)
          if( rho > scale_factor ) {
            p_map_x[x] = -1.0f;
            p_map_y[x] = -1.0f;
            continue;
          }
          double c = std::asin(rho / scale_factor);
          lat_p = std::asin(std::cos(c) * std::sin(lat0) - (dy * std::sin(c) * std::cos(lat0)) / rho);
          lon_p = lon0
              + std::atan2(dx * std::sin(c), rho * std::cos(lat0) * std::cos(c) + dy * std::sin(lat0) * std::sin(c));
        }
        else if( proj_type == c_alpha_test_routine::ProjectionStereographic ) {
          // Inverse stereographic projection relative to the central point (lon0, lat0)
          double c = 2.0 * std::atan(rho / scale_factor);
          lat_p = std::asin(std::cos(c) * std::sin(lat0) - (dy * std::sin(c) * std::cos(lat0)) / rho);
          lon_p = lon0 + std::atan2(dx * std::sin(c),
              rho * std::cos(lat0) * std::cos(c) + dy * std::sin(lat0) * std::sin(c));
        }
      }

      // Direct projection of a point on a sphere (lon_p, lat_p) onto a flat sensor (into the observer's system with librations)
      double cos_lat_p = std::cos(lat_p);
      double sin_lat_p = std::sin(lat_p);
      double d_lon = lon_p - l;

      // Coordinates on the unit sphere of the visible disk of the Moon
      double x_sph = cos_lat_p * std::sin(d_lon);
      double y_sph = sin_lat_p * std::cos(b) - cos_lat_p * std::sin(b) * std::cos(d_lon);
      double z_sph = sin_lat_p * std::sin(b) + cos_lat_p * std::cos(b) * std::cos(d_lon);

      if( z_sph < 0 ) { // A point beyond the horizon of the Moon
        p_map_x[x] = -1.0f;
        p_map_y[x] = -1.0f;
        continue;
      }

      // position on the Moon's disk in absolute pixels of the original full image
      double x_full_cam = x_sph * R_moon_pixels;
      double y_full_cam = -y_sph * R_moon_pixels;

      // Here we need to understand where this pixel is NOW located within the CROP.
      // To do this we first find where the center of our crop is in the full-frame system (lon0, lat0)
      double cos_lat0 = std::cos(lat0);
      double sin_lat0 = std::sin(lat0);
      double d_lon0 = lon0 - l;

      double x_center_sph = cos_lat0 * std::sin(d_lon0);
      double y_center_sph = sin_lat0 * std::cos(b) - cos_lat0 * std::sin(b) * std::cos(d_lon0);

      double x_center_full_cam = x_center_sph * R_moon_pixels;
      double y_center_full_cam = -y_center_sph * R_moon_pixels;

      // Offset of the current point relative to the center of our frame (WITHOUT taking into account camera rotation)
      double dx_cam = x_full_cam - x_center_full_cam;
      double dy_cam = y_full_cam - y_center_full_cam;

      // Apply camera rotation around the center of our frame
      double cos_cr = std::cos(cam_rot);
      double sin_cr = std::sin(cam_rot);

      double x_img = dx_cam * cos_cr - dy_cam * sin_cr + cx_src;
      double y_img = dx_cam * sin_cr + dy_cam * cos_cr + cy_src;

      p_map_x[x] = static_cast<float>(x_img);
      p_map_y[x] = static_cast<float>(y_img);
    }
  }
}

} // namespace

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "_Projection", ctx(&this_class::_Projection), "");
  ctlbind(ctls, "Lon [deg]", ctx(&this_class::_lon), "Selenographic Longiture in degreed");
  ctlbind(ctls, "Lat [deg]", ctx(&this_class::_lat), "Selenographic Latitude in degreed");
  ctlbind(ctls, "Libr. l [deg]", ctx(&this_class::_l), "Libration in longitude in degreed");
  ctlbind(ctls, "Libr. b [deg]", ctx(&this_class::_b), "Libration in Latitude in degreed");
  ctlbind(ctls, "_R_moon_pixels", ctx(&this_class::_R_moon_pixels), "");
  ctlbind_slider_spinbox(ctls, "Camera Orientation [deg]", ctx(&this_class::_camera_rotation), -180, 180, 0.1,
      "Orientation angle in degrees");
  ctlbind(ctls, "Resize mode", ctx(&this_class::_resizeMode), "Don't crop bounding box");
  ctlbind(ctls, "interpolation", ctx(&this_class::_interpolation), "");
  ctlbind(ctls, "border mode", ctx(&this_class::_borderMode), "");
  ctlbind(ctls, "border value", ctx(&this_class::_borderValue), "");
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _Projection);
    SERIALIZE_OPTION(settings, save, *this, _lon);
    SERIALIZE_OPTION(settings, save, *this, _lat);
    SERIALIZE_OPTION(settings, save, *this, _l);
    SERIALIZE_OPTION(settings, save, *this, _b);
    SERIALIZE_OPTION(settings, save, *this, _R_moon_pixels);
    SERIALIZE_OPTION(settings, save, *this, _camera_rotation);
    SERIALIZE_OPTION(settings, save, *this, _resizeMode);
    SERIALIZE_OPTION(settings, save, *this, _interpolation);
    SERIALIZE_OPTION(settings, save, *this, _borderMode);
    SERIALIZE_OPTION(settings, save, *this, _borderValue);
    return true;
  }
  return false;
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Mat src = image.getMat();
  const cv::Mat msk = mask.getMat();

  c_lunar_birdview_options opts;
  opts.lat = _lat;
  opts.lon = _lon;
  opts.l = _l;
  opts.b = _b;
  opts.camera_rotation = _camera_rotation;
  opts.interpolation = _interpolation;
  opts.borderMode = _borderMode;
  opts.borderValue = _borderValue;
  opts.resizeMode = _resizeMode;

  cv::Size dst_size = src.size() * 2;

  cv::Mat map_x, map_y;
  createMoonUnrollMaps(opts, src.size(), dst_size, _R_moon_pixels, map_x, map_y, _Projection);

  cv::remap(src, image, map_x, map_y,
      opts.interpolation,
      opts.borderMode,
      opts.borderValue);

  if( !mask.empty() ) {
    cv::remap(mask.getMat(), mask, map_x, map_y,
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));
  }

  return true;
}

