/*
 * c_saturn_derotation_remap.cc
 *
 *  Created on: Jun 4, 2026
 *      Author: amyznikov
 */

#include "c_saturn_derotation_remap.h"

void c_saturn_derotation_remap::set_opts(const c_saturn_derotation_remap_options & opts)
{
  _opts = opts;
}

const c_saturn_derotation_remap_options & c_saturn_derotation_remap::opts() const
{
  return _opts;
}

void c_saturn_derotation_remap::set_reference_pose(const cv::Size & image_size,
    const cv::Point2d & center,
    const cv::Vec3d & axes,
    const cv::Vec3d & pose)
{
  _image_size = image_size;
  _center = center;
  _axes = axes;
  _current_pose = _target_pose = pose;
  _Rcurrent = _Rtarget = build_ellipsoid_rotation(_target_pose);
}

void c_saturn_derotation_remap::compute_derotation_for_time(double deltat_sec, double wscale)
{
  static constexpr double default_rotation_period_sec = 10 * 3600. + 33 * 60. + 38;
  const double rotation_period_sec = _opts.saturn_rotation_period_sec > 0 ? _opts.saturn_rotation_period_sec : default_rotation_period_sec;
  const double rotation_angle_radians = CV_2PI * deltat_sec / rotation_period_sec;
  return compute_derotation_for_angle(rotation_angle_radians, wscale);
}

void c_saturn_derotation_remap::compute_derotation_for_angle(double longitude_rotation_radians, double wscale)
{
  _current_pose = cv::Vec3d(_target_pose(0) + longitude_rotation_radians, _target_pose(1), _target_pose(2));
  _Rcurrent = build_ellipsoid_rotation(_current_pose);

  compute_ellipsoid_zrotation_remap(_image_size,
      _center,
      _axes,
      _Rcurrent,
      _Rtarget,
      _rmap,
      _wmap,
      _rmask,
      wscale);
}

