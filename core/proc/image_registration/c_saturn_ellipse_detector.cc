/*
 * c_saturn_ellipse_detector.cc
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#include "c_saturn_ellipse_detector.h"
#include <core/proc/ellipsoid.h>
#include <core/proc/pose.h>
#include <core/debug.h>

bool c_saturn_ellipse_detector::detect(cv::InputArray image, cv::InputArray mask)
{
  static constexpr double radius_ratio =
      54400. / 60300.;

  const cv::Size image_size =
      image.size();

  bool saturn_detected = false;

  saturn_detected =
      detect_saturn(image, options_.se_close_radius,
          saturn_bounding_box_, saturn_mask_);

  if ( !saturn_detected ) {
    CF_ERROR("detect_saturn() fails");
    return false;
  }

  double & A = ellipsoid_size_(0);
  double & B = ellipsoid_size_(1);
  double & C = ellipsoid_size_(2);

  C = saturn_bounding_box_.size.height / 2;
  A = B = (C / radius_ratio);
  ring_radius_ = saturn_bounding_box_.size.width / 2;
  center_ = saturn_bounding_box_.center;
  ellipsoid_rotation_ = cv::Vec3d(options_.xrotation, saturn_bounding_box_.angle, 0) * CV_PI / 180;

  const cv::Matx33d R =
      build_rotation2(ellipsoid_rotation_);

  planetary_disk_ellipse_ =
      ellipsoid_bbox(center_, A, B, C, R.t());

  planetary_disk_ellipse_mask_.create(image.size());
  planetary_disk_ellipse_mask_.setTo(0);

  draw_ellipse(planetary_disk_ellipse_mask_,
      planetary_disk_ellipse_,
      cv::Scalar::all(255), -1,
      cv::LINE_8);

  return true;
}
