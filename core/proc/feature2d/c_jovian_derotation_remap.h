/*
 * c_jovian_derotation_remap.h
 *
 *  Created on: May 25, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_jovian_derotation_remap_h__
#define __c_jovian_derotation_remap_h__

#include <opencv2/opencv.hpp>
#include <core/proc/feature2d/ellipsoid.h>

class c_jovian_derotation_remap
{
public:
  void set_reference_pose(const cv::Size & image_size, const cv::Point2d & center, const cv::Vec3d & axes, const cv::Vec3d & pose);
  void compute_derotation_for_angle(double longitude_rotation_radians, double wscale = 1);
  void compute_derotation_for_time(double deltat_msec, double wscale = 1);

  const cv::Mat2f & rmap() const
  {
    return _rmap;
  }

  const cv::Mat1f & wmap() const
  {
    return _wmap;
  }

  const cv::Mat1b & rmask() const
  {
    return _rmask;
  }

  const cv::Mat1f & rcounter() const
  {
    return _rcounter;
  }

  const cv::Size & image_size() const
  {
    return _image_size;
  }

  const cv::Point2d & center() const
  {
    return _center;
  }

  const cv::Vec3d & axes() const
  {
    return _axes;
  }

  const cv::Vec3d & current_pose() const
  {
    return _current_pose;
  }

  const cv::Vec3d & target_pose() const
  {
    return _target_pose;
  }

  const cv::Matx33d & Rcurrent() const
  {
    return _Rcurrent;
  }

  const cv::Matx33d & Rtarget() const
  {
    return _Rtarget;
  }

protected:
  cv::Size _image_size;
  cv::Point2d _center;
  cv::Vec3d _axes;
  cv::Vec3d _current_pose;
  cv::Vec3d _target_pose;

  cv::Matx33d _Rcurrent;
  cv::Matx33d _Rtarget;

  cv::Mat2f _rmap;
  cv::Mat1f _wmap;
  cv::Mat1b _rmask;
  cv::Mat1f _rcounter;
};

#endif /* __c_jovian_derotation_remap_h__ */
