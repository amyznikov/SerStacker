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
#include <core/proc/camera_calibration/camera_calibration.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/settings/opencv_settings.h>
#include <core/ctrlbind/ctrlbind.h>

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
    c_camera_intrinsics camera_intrinsics;
    c_lm_camera_pose_options camera_pose;
    cv::Vec3f initial_translation = cv::Vec3f(0, 0, 1);
    cv::Vec3f initial_rotation = cv::Vec3f(0, 0, 0); // euler angles in degrees

//    cv::Matx33d camera_matrix = // Dummy stub from KITTI
//        cv::Matx33d(
//            7.215377e+02, 0.000000e+00, 6.095593e+02,
//            0.000000e+00, 7.215377e+02, 1.728540e+02,
//            0.000000e+00, 0.000000e+00, 1.000000e+00);

  } epipolar_derotation;

};


bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<cv::DMatch> & sparse_matches,
    const c_estimate_image_transform_options & opts);

bool estimate_translation(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_total_euclidean_transform(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_translation_and_rotation(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_euclidean_transform(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_affine_transform(c_affine_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_homography_transform(c_homography_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_semi_quadratic_transform(c_semi_quadratic_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_quadratic_transform(c_quadratic_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool estimate_epipolar_derotation(c_epipolar_derotation_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts);

bool serialize_image_transform_estimation_options(c_config_setting section, bool save,
    c_estimate_image_transform_options & opts);

inline bool save_settings(c_config_setting section, const c_estimate_image_transform_options & opts)
{
  return serialize_image_transform_estimation_options(section, true,
      const_cast<c_estimate_image_transform_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_estimate_image_transform_options * opts)
{
  return serialize_image_transform_estimation_options(section, false, *opts);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_estimate_image_transform_options> & ctx)
{
  using S = c_estimate_image_transform_options;
  ctlbind_expandable_group(ctls, "Translation", [&, ctx = CTL_CONTEXT(ctx, translation)]() {
    ctlbind(ctls, "max_iterations", CTL_CONTEXT(ctx, max_iterations), "");
    ctlbind(ctls, "rmse_factor", CTL_CONTEXT(ctx, rmse_factor), "");
  });

  // translation + rotation
  ctlbind_expandable_group(ctls, "Euclidean", [&, ctx = CTL_CONTEXT(ctx, euclidean)]() {
    ctlbind(ctls, "max_iterations", CTL_CONTEXT(ctx, max_iterations), "");
    ctlbind(ctls, "rmse_threshold", CTL_CONTEXT(ctx, rmse_threshold ), "");
  });

  ctlbind_expandable_group(ctls, "ScaledEuclidean", [&, ctx = CTL_CONTEXT(ctx, scaled_euclidean)]() {
    ctlbind(ctls, "method", CTL_CONTEXT(ctx, method), "");
    ctlbind(ctls, "maxIters", CTL_CONTEXT(ctx, maxIters), "");
    ctlbind(ctls, "ransacReprojThreshold", CTL_CONTEXT(ctx, ransacReprojThreshold), "");
    ctlbind(ctls, "confidence", CTL_CONTEXT(ctx, confidence), "");
    ctlbind(ctls, "refineIters", CTL_CONTEXT(ctx, refineIters), "");
  });

  ctlbind_expandable_group(ctls, "Affine", [&, ctx = CTL_CONTEXT(ctx, affine)]() {
    ctlbind(ctls, "method", CTL_CONTEXT(ctx, method), "");
    ctlbind(ctls, "maxIters", CTL_CONTEXT(ctx, maxIters), "");
    ctlbind(ctls, "ransacReprojThreshold", CTL_CONTEXT(ctx, ransacReprojThreshold), "");
    ctlbind(ctls, "confidence", CTL_CONTEXT(ctx, confidence), "");
    ctlbind(ctls, "refineIters", CTL_CONTEXT(ctx, refineIters), "");
  });

  ctlbind_expandable_group(ctls, "Homography", [&, ctx = CTL_CONTEXT(ctx, homography)]() {
    ctlbind(ctls, "method", CTL_CONTEXT(ctx, method), "");
    ctlbind(ctls, "maxIters", CTL_CONTEXT(ctx, maxIters), "");
    ctlbind(ctls, "ransacReprojThreshold", CTL_CONTEXT(ctx, ransacReprojThreshold), "");
    ctlbind(ctls, "confidence", CTL_CONTEXT(ctx, confidence), "");
  });

  ctlbind_expandable_group(ctls, "SemiQuadratic", [&, ctx = CTL_CONTEXT(ctx, semi_quadratic)]() {
    ctlbind(ctls, "rmse_factor", CTL_CONTEXT(ctx, rmse_factor), "");
  });

  ctlbind_expandable_group(ctls, "Quadratic", [&, ctx = CTL_CONTEXT(ctx, quadratic)]() {
    ctlbind(ctls, "rmse_factor", CTL_CONTEXT(ctx, rmse_factor), "");
  });

  ctlbind_expandable_group(ctls, "Epipolar derotation", [&, ctx = CTL_CONTEXT(ctx, epipolar_derotation)]() {
    ctlbind(ctls, "initial_translation", CTL_CONTEXT(ctx, initial_translation), "");
    ctlbind(ctls, "initial_rotation", CTL_CONTEXT(ctx, initial_rotation), "");
    ctlbind_expandable_group(ctls, "Camera", CTL_CONTEXT(ctx, camera_intrinsics));
    ctlbind_expandable_group(ctls, "Pose estimation", CTL_CONTEXT(ctx, camera_pose));
  });


}

#endif /* __estimate_image_transform_h__ */
