/*
 * c_homography_test_routine.cc
 *
 *  Created on: Mar 6, 2023
 *      Author: amyznikov
 */

#include "c_homography_test_routine.h"
#include <core/proc/camera_calibration/camera_pose.h>

bool c_homography_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  // FIXME: Hardcoded Camera Matrix
  // Extracted from P_rect_00 of KITTI calib_cam_to_cam.txt
  // .image_size = cv::Size(1242, 375),
  static const cv::Matx33d camera_matrix =
      cv::Matx33d(
          7.215377e+02, 0.000000e+00, 6.095593e+02,
          0.000000e+00, 7.215377e+02, 1.728540e+02,
          0.000000e+00, 0.000000e+00, 1.000000e+00);


  static const cv::Matx33d camera_matrix_inv =
      camera_matrix.inv();

  const cv::Matx33d R =
      build_rotation(rx_ * CV_PI / 180,
          ry_ * CV_PI / 180,
          rz_ * CV_PI / 180);

  const cv::Matx33d H =
      camera_matrix * R * camera_matrix_inv;

  cv::warpPerspective(image, image,
      H,
      image.size(),
      cv::INTER_LINEAR, // Note that cv::INTER_LINEAR can introduce some blur on kitti images
      cv::BORDER_CONSTANT);

  if( !mask.empty() ) {

    cv::warpPerspective(mask, mask,
        H,
        image.size(),
        cv::INTER_LINEAR, // Note that cv::INTER_LINEAR can introduce some blur on kitti images
        cv::BORDER_CONSTANT);

    cv::compare(mask, 255, mask, cv::CMP_GE);
  }

  return true;
}
