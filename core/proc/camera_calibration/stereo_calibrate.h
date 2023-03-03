/*
 * stereo_calibrate.h
 *
 *  Created on: Mar 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __stereo_calibrate_h__
#define __stereo_calibrate_h__

#include "camera_calibration.h"


/**
 * Flags for cv::stereoCalibrate()
 */
enum STEREO_CALIBRATION_FLAGS
{
  STEREO_CALIB_FIX_INTRINSIC = cv::CALIB_FIX_INTRINSIC,
  STEREO_CALIB_USE_INTRINSIC_GUESS = cv::CALIB_USE_INTRINSIC_GUESS,
  STEREO_CALIB_USE_EXTRINSIC_GUESS = cv::CALIB_USE_EXTRINSIC_GUESS,
  STEREO_CALIB_FIX_PRINCIPAL_POINT = cv::CALIB_FIX_PRINCIPAL_POINT,
  STEREO_CALIB_FIX_FOCAL_LENGTH = cv::CALIB_FIX_FOCAL_LENGTH,
  STEREO_CALIB_FIX_ASPECT_RATIO = cv::CALIB_FIX_ASPECT_RATIO,
  STEREO_CALIB_SAME_FOCAL_LENGTH = cv::CALIB_SAME_FOCAL_LENGTH,
  STEREO_CALIB_ZERO_TANGENT_DIST = cv::CALIB_ZERO_TANGENT_DIST,
  STEREO_CALIB_FIX_K1 = cv::CALIB_FIX_K1,
  STEREO_CALIB_FIX_K2 = cv::CALIB_FIX_K2,
  STEREO_CALIB_FIX_K3 = cv::CALIB_FIX_K3,
  STEREO_CALIB_FIX_K4 = cv::CALIB_FIX_K4,
  STEREO_CALIB_FIX_K5 = cv::CALIB_FIX_K5,
  STEREO_CALIB_FIX_K6 = cv::CALIB_FIX_K6,
  STEREO_CALIB_RATIONAL_MODEL = cv::CALIB_RATIONAL_MODEL,
  STEREO_CALIB_THIN_PRISM_MODEL = cv::CALIB_THIN_PRISM_MODEL,
  STEREO_CALIB_FIX_S1_S2_S3_S4 = cv::CALIB_FIX_S1_S2_S3_S4,
  STEREO_CALIB_TILTED_MODEL = cv::CALIB_TILTED_MODEL,
  STEREO_CALIB_FIX_TAUX_TAUY = cv::CALIB_FIX_TAUX_TAUY,
};


/**
 *  Wrapper around of cv::initCameraMatrix2D()
 *  Finds an initial camera intrinsic matrix from 3D-2D point correspondences.
 * */
bool init_camera_intrinsics(c_stereo_camera_intrinsics & intrinsics,
    const std::vector<std::vector<cv::Point3f>> & objectPoints,
    const std::vector<std::vector<cv::Point2f>> & imagePoints1,
    const std::vector<std::vector<cv::Point2f>> & imagePoints2,
    const cv::Size & imageSize,
    double aspectRatio = 1.0);


/**
 * camera_matrix_from_stereo_projection_matrix()
 *
 * Extract camera matrix from stereo projection matrix P returned from cv::stereoRectity()
 */
template<class T>
inline cv::Matx<T, 3, 3> camera_matrix_from_stereo_projection_matrix(const cv::Matx<T, 3, 4> & P)
{
  return cv::Matx<T, 3, 3>(
      P(0, 0), P(0, 1), P(0, 2),
      P(1, 0), P(1, 1), P(1, 2),
      P(2, 0), P(2, 1), P(2, 2));
}


/**
 *  Wrapper around of cv::stereoCalibrateExtended().
 *  Return rmse or -1 on error
 */
double stereo_calibrate(cv::InputArrayOfArrays objectPoints,
    cv::InputArrayOfArrays imagePoints1, cv::InputArrayOfArrays imagePoints2,
    c_stereo_camera_intrinsics & intrinsics,
    c_stereo_camera_extrinsics & extrinsics,
    int flags = cv::CALIB_USE_INTRINSIC_GUESS,
    const cv::TermCriteria & term = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6),
    /* out, opt */ cv::Matx33d * E = nullptr,
    /* out, opt */ cv::Matx33d * F = nullptr,
    /* out, opt */ std::vector<cv::Vec3d> * rvecs = nullptr,
    /* out, opt */ std::vector<cv::Vec3d> * tvecs = nullptr,
    /* out, opt */ cv::Mat1d * perViewErrors = nullptr );




/**
 * create_stereo_rectification()
 *
 * Uses cv::stereoRectify() followed by cv::initUndistortRectifyMap() to
 * precompute remaps for distortion correction and stereo rectification,
 *
 * Computes new intrinsics and extrinsics after rectification
 * These intrinsics and extrinsics are supposed to be valid for images
 * remapped using the maps.
 *
 * Input args:
 *
 *    dstImageSize  :
 *      The size of the image after undistortion and stereo rectifixation.
 *      Can be empty, in this case the size will the same as specified in
 *      input stereo camera intrinsics.
 *
 *    intrinsics    :
 *      Stereo camera intrinsics before rectification.
 *
 *    extrinsics    :
 *      Stereo camera extrinsics before rectification.
 *
 * Output args:
 *
 *    new_intrinsics:
 *      Optional pointer to store new stereo camera intrinsics after rectification.
 *
 *    new_extrinsics:
 *      Optional pointer to store new stereo camera extrinsics after rectification.
 *
 *    outputR[2]    :
 *      Optional pointer to store output 3x3 rectification transform (rotation matrix)
 *      for the both cameras. Each matrix brings points given in the unrectified camera's
 *      coordinate system to points in the rectified camera's coordinate system.
 *      If not null then it must point to an array of two 3x3 matrices.
 *
 *    outputP[2]    :
 *      Optional pointer to store output 3x4 projection matrices
 *      for the both cameras. Each matrix projects points given in the rectified camera
 *      coordinate system into the rectified camera's image.
 *      If not null then it must point to an array of two 3x4 matrices.
 *
 *   outputQ        :
 *      Optional pointer to store output 4x4 perspective transformation
 *      (disparity-to-depth mapping) matrix (see @ref reprojectImageTo3D).
 *
 *  outputValidRoi[2]:
 *      Optional output rectangles inside the rectified images where all the pixels
 *      are valid. If alpha=0, the ROIs cover the whole images. Otherwise, they are
 *      likely to be smaller.
 *      If not null then it must point to an array of two cv::Rect structures.
 *
 * */

bool create_stereo_rectification(const cv::Size & dstImageSize,
    const c_stereo_camera_intrinsics & intrinsics,
    const c_stereo_camera_extrinsics & extrinsics,
    double alpha,
    /* out, opt*/cv::Mat2f rmaps[2]  = nullptr,
    /* out, opt*/c_stereo_camera_intrinsics * new_intrinsics = nullptr,
    /* out, opt*/c_stereo_camera_extrinsics * new_extrinsics = nullptr,
    /* out, opt*/cv::Matx33d outputR[2] = nullptr,
    /* out, opt*/cv::Matx34d outputP[2] = nullptr,
    /* out, opt*/cv::Matx44d * outputQ = nullptr,
    /* out, opt*/cv::Rect outputValidRoi[2] = nullptr);



#endif /* __stereo_calibrate_h__ */
