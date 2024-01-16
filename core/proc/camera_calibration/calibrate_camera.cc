/*
 * calibrate_camera.cc
 *
 *  Created on: Mar 3, 2023
 *      Author: amyznikov
 */
#include "calibrate_camera.h"
#include <core/ssprintf.h>
#include <core/debug.h>

#define CATCH(fn) \
  catch( const cv::Exception &e ) { \
    CF_ERROR("OpenCV exception %s in %s():\n" \
        "%s\n" \
        "%s() : %d\n" \
        "file : %s\n", \
        fn, \
        __func__, \
        e.err.c_str(), \
        e.func.c_str(), \
        e.line, \
        e.file.c_str() \
        ); \
  } \
  catch( const std::exception &e ) { \
    CF_ERROR("std::exception %s in %s(): %s\n", fn, __func__, e.what()); \
  } \
  catch( ... ) { \
    CF_ERROR("Unknown exception %s in %s()\n", fn, __func__); \
  }



template<>
const c_enum_member* members_of<CAMERA_CALIBRATION_FLAGS>()
{
  static const c_enum_member members[] = {
      { CAMERA_CALIB_USE_INTRINSIC_GUESS, "USE_INTRINSIC_GUESS", "" },
      { CAMERA_CALIB_FIX_ASPECT_RATIO, "FIX_ASPECT_RATIO", "" },
      { CAMERA_CALIB_FIX_PRINCIPAL_POINT, "FIX_PRINCIPAL_POINT", "" },
      { CAMERA_CALIB_ZERO_TANGENT_DIST, "ZERO_TANGENT_DIST", "" },
      { CAMERA_CALIB_FIX_FOCAL_LENGTH, "FIX_FOCAL_LENGTH", "" },
      { CAMERA_CALIB_FIX_K1, "FIX_K1", "" },
      { CAMERA_CALIB_FIX_K2, "FIX_K2", "" },
      { CAMERA_CALIB_FIX_K3, "FIX_K3", "" },
      { CAMERA_CALIB_FIX_K4, "FIX_K4", "" },
      { CAMERA_CALIB_FIX_K5, "FIX_K5", "" },
      { CAMERA_CALIB_FIX_K6, "FIX_K6", "" },
      { CAMERA_CALIB_RATIONAL_MODEL, "RATIONAL_MODEL", "" },
      { CAMERA_CALIB_THIN_PRISM_MODEL, "THIN_PRISM_MODEL", "" },
      { CAMERA_CALIB_FIX_S1_S2_S3_S4, "FIX_S1_S2_S3_S4", "" },
      { CAMERA_CALIB_TILTED_MODEL, "TILTED_MODEL", "" },
      { CAMERA_CALIB_FIX_TAUX_TAUY, "FIX_TAUX_TAUY", "" },
      { CAMERA_CALIB_USE_QR, "USE_QR",
          "Use QR instead of SVD decomposition for solving. Faster but potentially less precise" }, //!<
      { CAMERA_CALIB_FIX_TANGENT_DIST, "FIX_TANGENT_DIST", "" },
      { CAMERA_CALIB_USE_LU, "USE_LU",
          "Use LU instead of SVD decomposition for solving. much faster but potentially less precise" },
      { 0 }
  };

  return members;
}



/**
 *  Wrapper around of cv::initCameraMatrix2D()
 *  Finds an initial camera intrinsic matrix from 3D-2D point correspondences.
 * */
bool init_camera_intrinsics(c_camera_intrinsics & intrinsics,
    const std::vector<std::vector<cv::Point3f>> & objectPoints,
    const std::vector<std::vector<cv::Point2f>> & imagePoints,
    const cv::Size & imageSize,
    double aspectRatio)
{

  bool fOK = false;

  try {

    intrinsics.image_size =
        imageSize;

    intrinsics.camera_matrix =
        cv::initCameraMatrix2D(objectPoints,
            imagePoints,
            imageSize,
            aspectRatio);

    fOK = true;
  }
  CATCH("from cv::initCameraMatrix2D()");

  return fOK;
}

/**
 *  Wrapper around of cv::calibrateCamera()
 *  Return rmse or -1 on error
 */
double calibrate_camera(cv::InputArrayOfArrays objectPoints,
    cv::InputArrayOfArrays imagePoints,
    c_camera_intrinsics & intrinsics,
    int flags,
    const cv::TermCriteria & term,
    /* out, opt */std::vector<cv::Vec3d> * rvecs,
    /* out, opt */std::vector<cv::Vec3d> * tvecs,
    /* out, opt */cv::Mat1d * stdDeviationsIntrinsics,
    /* out, opt */cv::Mat1d * stdDeviationsExtrinsics,
    /* out, opt */cv::Mat1d * perViewErrors)
{
  double rmse = -1;

  try {
    rmse =
        cv::calibrateCamera(objectPoints,
            imagePoints,
            intrinsics.image_size,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            rvecs ? *rvecs : cv::noArray(),
            tvecs ? *tvecs : cv::noArray(),
            stdDeviationsIntrinsics ? *stdDeviationsIntrinsics : cv::noArray(),
            stdDeviationsExtrinsics ? *stdDeviationsExtrinsics : cv::noArray(),
            perViewErrors ? *perViewErrors : cv::noArray(),
            flags,
            term);
  }
  CATCH("from cv::calibrateCamera()");

  return rmse;
}
