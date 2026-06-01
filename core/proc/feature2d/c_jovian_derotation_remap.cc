/*
 * c_jovian_derotation_remap.cc
 *
 *  Created on: May 25, 2026
 *      Author: amyznikov
 */

#include "c_jovian_derotation_remap.h"

void c_jovian_derotation_remap::set_reference_pose(const cv::Size & image_size,
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

void c_jovian_derotation_remap::compute_derotation_for_time(double deltat_msec, double wscale)
{
  // Jupiter daily rotation periods are
  // System III : 9h 55m 30s.
  // System II  : 9ч 55м 40.632s
  // System I   : 9ч 50м 30.003s
  //static constexpr double rotation_period_sec = 9. * 3600 + 55. * 60 + 30.;
  static constexpr double rotation_period_sec = 9. * 3600 + 55. * 60 + 40.632;
  const double rotation_angle_deg = 0.360 * deltat_msec / rotation_period_sec;
  const double rotation_angle_radians = rotation_angle_deg * CV_PI / 180;
  return compute_derotation_for_angle(rotation_angle_radians, wscale);
}



void c_jovian_derotation_remap::compute_derotation_for_angle(double longitude_rotation_radians, double wscale)
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

  //compute_ellipsoid_zrotation_wmap(_center, _axes,  _target_pose, _rmap, _wmap);
}

