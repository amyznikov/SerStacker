/*
 * calibrate_camera.h
 *
 *  Created on: Mar 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __calibrate_camera_h__
#define __calibrate_camera_h__

#include "camera_calibration.h"

/**
 * Flags for cv::calibrateCamera()
 */
enum CAMERA_CALIBRATION_FLAGS
{
  CAMERA_CALIB_USE_INTRINSIC_GUESS = cv::CALIB_USE_INTRINSIC_GUESS,
  CAMERA_CALIB_FIX_ASPECT_RATIO = cv::CALIB_FIX_ASPECT_RATIO,
  CAMERA_CALIB_FIX_PRINCIPAL_POINT = cv::CALIB_FIX_PRINCIPAL_POINT,
  CAMERA_CALIB_ZERO_TANGENT_DIST = cv::CALIB_ZERO_TANGENT_DIST,
  CAMERA_CALIB_FIX_FOCAL_LENGTH = cv::CALIB_FIX_FOCAL_LENGTH,
  CAMERA_CALIB_FIX_K1 = cv::CALIB_FIX_K1,
  CAMERA_CALIB_FIX_K2 = cv::CALIB_FIX_K2,
  CAMERA_CALIB_FIX_K3 = cv::CALIB_FIX_K3,
  CAMERA_CALIB_FIX_K4 = cv::CALIB_FIX_K4,
  CAMERA_CALIB_FIX_K5 = cv::CALIB_FIX_K5,
  CAMERA_CALIB_FIX_K6 = cv::CALIB_FIX_K6,
  CAMERA_CALIB_RATIONAL_MODEL = cv::CALIB_RATIONAL_MODEL,
  CAMERA_CALIB_THIN_PRISM_MODEL = cv::CALIB_THIN_PRISM_MODEL,
  CAMERA_CALIB_FIX_S1_S2_S3_S4 = cv::CALIB_FIX_S1_S2_S3_S4,
  CAMERA_CALIB_TILTED_MODEL = cv::CALIB_TILTED_MODEL,
  CAMERA_CALIB_FIX_TAUX_TAUY = cv::CALIB_FIX_TAUX_TAUY,
  CAMERA_CALIB_USE_QR = cv::CALIB_USE_QR, //!< use QR instead of SVD decomposition for solving. Faster but potentially less precise
  CAMERA_CALIB_FIX_TANGENT_DIST = cv::CALIB_FIX_TANGENT_DIST,
  CAMERA_CALIB_USE_LU = cv::CALIB_USE_LU, //!< use LU instead of SVD decomposition for solving. much faster but potentially less precise
};



/**
 *  Wrapper around of cv::initCameraMatrix2D()
 *  Finds an initial camera intrinsic matrix from 3D-2D point correspondences.
 * */
bool init_camera_intrinsics(c_camera_intrinsics & intrinsics,
    const std::vector<std::vector<cv::Point3f>> & objectPoints,
    const std::vector<std::vector<cv::Point2f>> & imagePoints,
    const cv::Size & imageSize,
    double aspectRatio = 1.0);


/**
 *  Wrapper around of cv::calibrateCamera()
 *  Return rmse or -1 on error
 */
double calibrate_camera(cv::InputArrayOfArrays objectPoints,
    cv::InputArrayOfArrays imagePoints,
    c_camera_intrinsics & intrinsics,
    int flags = cv::CALIB_USE_INTRINSIC_GUESS,
    const cv::TermCriteria & term = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6),
    /* out, opt */ std::vector<cv::Vec3d> * rvecs = nullptr,
    /* out, opt */ std::vector<cv::Vec3d> * tvecs = nullptr,
    /* out, opt */ cv::Mat1d * stdDeviationsIntrinsics = nullptr,
    /* out, opt */ cv::Mat1d * stdDeviationsExtrinsics = nullptr,
    /* out, opt */ cv::Mat1d * perViewErrors = nullptr );




#endif /* __calibrate_camera_h__ */
