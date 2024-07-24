/*
 * c_saturn_ellipse_detector.h
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_saturn_ellipse_detector_h__
#define __c_saturn_ellipse_detector_h__

#include <opencv2/opencv.hpp>

struct c_saturn_ellipse_detector_options
{
  double xrotation = 90; // planet inclination to the ray of view, [deg]
  double stdev_factor = 0.5;
  int se_close_radius = 3;
};

class c_saturn_ellipse_detector
{
public:

  bool detect(cv::InputArray _image, cv::InputArray mask = cv::noArray());

  c_saturn_ellipse_detector_options & options()
  {
    return options_;
  }

  const c_saturn_ellipse_detector_options & options() const
  {
    return options_;
  }

  const cv::Mat1b & saturn_mask() const
  {
    return saturn_mask_;
  }

  const cv::RotatedRect & saturn_bounding_box() const
  {
    return saturn_bounding_box_;
  }

  const cv::RotatedRect & planetary_disk_ellipse() const
  {
    return planetary_disk_ellipse_;
  }

  const cv::Mat1b & planetary_disk_ellipse_mask() const
  {
    return planetary_disk_ellipse_mask_;
  }

  const cv::Point2f & center() const
  {
    return center_;
  }

  const cv::Vec3d & ellipsoid_size() const
  {
    return ellipsoid_size_;
  }

  const cv::Vec3d & ellipsoid_rotation() const
  {
    return ellipsoid_rotation_;
  }


protected:
  c_saturn_ellipse_detector_options options_;
  cv::Mat1b saturn_mask_;
  cv::RotatedRect saturn_bounding_box_;
  cv::RotatedRect planetary_disk_ellipse_;
  cv::Mat1b planetary_disk_ellipse_mask_;
  double ring_radius_ = 0;
  cv::Point2f center_;
  cv::Vec3d ellipsoid_rotation_;
  cv::Vec3d ellipsoid_size_;
};

#endif /* __c_saturn_ellipse_detector_h__ */
