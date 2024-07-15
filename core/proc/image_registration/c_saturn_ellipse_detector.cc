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

  double A, B, C, ring_radius;
  cv::Point2f center;
  cv::Vec3d rotation;

  saturn_detected =
      detect_saturn(image, options_.se_close_radius,
          saturn_bounding_box_, saturn_mask_);

  if ( !saturn_detected ) {
    CF_ERROR("detect_saturn() fails");
    return false;
  }

  C = saturn_bounding_box_.size.height / 2;
  A = B = C / radius_ratio;
  ring_radius = saturn_bounding_box_.size.width / 2;
  center = saturn_bounding_box_.center;
  //rotation = cv::Vec3d(orientation_(0), saturn_bounding_box_.angle, orientation_(2)) * CV_PI / 180; //  orientation() * CV_PI / 180;


  return false;
}
