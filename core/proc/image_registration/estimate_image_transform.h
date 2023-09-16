/*
 * estimate_image_transform.h
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __estimate_image_transform_h__
#define __estimate_image_transform_h__

#include "c_image_transform.h"
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/settings/opencv_settings.h>

enum ROBUST_METHOD
{
  ROBUST_METHOD_RANSAC = cv::RANSAC,
  ROBUST_METHOD_LMEDS = cv::LMEDS,
  ROBUST_METHOD_RHO = cv::RHO,
};

struct c_estimate_image_transform_options
{
  struct {
    // parameters for estimate_translation()
    double rmse_factor = 3;
    int max_iterations = 10;
  } translation;

  struct {
    // parameters for estimate_translation_and_rotation()
    double rmse_threshold = 1e-6;
    int max_iterations = 10;
  } euclidean;

  struct {
    // parameters for estimate_total_euclidean_transform()
    double ransacReprojThreshold = 3;
    double confidence = 0.99;
    ROBUST_METHOD method = ROBUST_METHOD_LMEDS;
    int maxIters = 2000;
    int refineIters = 10;
  } scaled_euclidean;

  struct {
    // parameters for estimate_affine_transform()
    ROBUST_METHOD method = ROBUST_METHOD_LMEDS;
    double ransacReprojThreshold = 3;
    int maxIters = 2000;
    double confidence = 0.99;
    int refineIters = 10;
  } affine;


  struct {
    // parameters for estimate_homography_transform()
    ROBUST_METHOD method = ROBUST_METHOD_LMEDS;
    double ransacReprojThreshold = 3;
    int maxIters = 2000;
    double confidence = 0.995;
  } homography;

  struct {
    // parameters for estimate_semi_quadratic_transform()
    double rmse_factor = 3;
  } semi_quadratic;

  struct {
    // parameters for estimate_quadratic_transform()
    double rmse_factor = 3;
  } quadratic;


  struct {
    // parameters for estimate_epipolar_derotation()
    c_lm_camera_pose_options camera_pose;

    cv::Matx33d camera_matrix = // Dummy stub from KITTI
        cv::Matx33d(
            7.215377e+02, 0.000000e+00, 6.095593e+02,
            0.000000e+00, 7.215377e+02, 1.728540e+02,
            0.000000e+00, 0.000000e+00, 1.000000e+00);

  } epipolar_derotation;

};



bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_,
    const c_estimate_image_transform_options * options = nullptr);


bool save_settings(c_config_setting, const c_estimate_image_transform_options& opts);
bool load_settings(c_config_setting, c_estimate_image_transform_options * opts);

#endif /* __estimate_image_transform_h__ */
