/*
 * stereo_calibrate.cc
 *
 *  Created on: Mar 3, 2023
 *      Author: amyznikov
 */


#include "stereo_calibrate.h"
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
const c_enum_member* members_of<STEREO_CALIBRATION_FLAGS>()
{
  static constexpr c_enum_member members[] = {
      {STEREO_CALIB_FIX_INTRINSIC, "FIX_INTRINSIC","Fix cameraMatrix? and distCoeffs? so that only R, T, E, and F matrices are estimated."},
      {STEREO_CALIB_USE_INTRINSIC_GUESS, "USE_INTRINSIC_GUESS"," Optimize some or all of the intrinsic parameters according to the specified flags. Initial values are provided by the user."},
      {STEREO_CALIB_USE_EXTRINSIC_GUESS, "USE_EXTRINSIC_GUESS"," R and T contain valid initial values that are optimized further. Otherwise R and T are initialized to the median value of the pattern views (each dimension separately)."},
      {STEREO_CALIB_FIX_PRINCIPAL_POINT, "FIX_PRINCIPAL_POINT"," Fix the principal points during the optimization."},
      {STEREO_CALIB_FIX_FOCAL_LENGTH, "FIX_FOCAL_LENGTH"," Fix f(j)x and f(j)y ."},
      {STEREO_CALIB_FIX_ASPECT_RATIO, "FIX_ASPECT_RATIO"," Optimize f(j)y . Fix the ratio f(j)x/f(j)y"},
      {STEREO_CALIB_SAME_FOCAL_LENGTH, "SAME_FOCAL_LENGTH"," Enforce f(0)x=f(1)x and f(0)y=f(1)y ."},
      {STEREO_CALIB_ZERO_TANGENT_DIST, "ZERO_TANGENT_DIST"," Set tangential distortion coefficients for each camera to zeros and fix there."},
      {STEREO_CALIB_FIX_K1, "FIX_K1","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K2, "FIX_K2","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K3, "FIX_K3","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K4, "FIX_K4","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K5, "FIX_K5","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K6, "FIX_K6","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_RATIONAL_MODEL, "RATIONAL_MODEL"," Enable coefficients k4, k5, and k6. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the rational model and return 8 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients."},
      {STEREO_CALIB_THIN_PRISM_MODEL, "THIN_PRISM_MODEL"," Coefficients s1, s2, s3 and s4 are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the thin prism model and return 12 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients."},
      {STEREO_CALIB_FIX_S1_S2_S3_S4, "FIX_S1_S2_S3_S4"," The thin prism distortion coefficients are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_TILTED_MODEL, "TILTED_MODEL"," Coefficients tauX and tauY are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the tilted sensor model and return 14 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients."},
      {STEREO_CALIB_FIX_TAUX_TAUY, "FIX_TAUX_TAUY"," The coefficients of the tilted sensor model are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      { 0 }
  };

  return members;
}



/**
 *  Wrapper around of cv::initCameraMatrix2D()
 *  Finds an initial camera intrinsic matrix from 3D-2D point correspondences.
 * */
bool init_camera_intrinsics(c_stereo_camera_intrinsics & intrinsics,
    const std::vector<std::vector<cv::Point3f>> & objectPoints,
    const std::vector<std::vector<cv::Point2f>> & imagePoints1,
    const std::vector<std::vector<cv::Point2f>> & imagePoints2,
    const cv::Size & imageSize,
    double aspectRatio)
{

  const std::vector<std::vector<cv::Point2f>> *imagePoints[2] = {
      &imagePoints1,
      &imagePoints2
  };

  bool fOK = false;

  try {

    for( int i = 0; i < 2; ++i ) {

      intrinsics.camera[i].image_size =
          imageSize;

      intrinsics.camera[i].camera_matrix =
          cv::initCameraMatrix2D(objectPoints,
              *imagePoints[i],
              imageSize,
              aspectRatio);

    }

    fOK = true;
  }
  CATCH("from cv::initCameraMatrix2D()");

  return fOK;
}




/**
 *  Wrapper around of cv::stereoCalibrateExtended()
 *  Return rmse or -1 on error
 */

double stereo_calibrate(cv::InputArrayOfArrays objectPoints,
    cv::InputArrayOfArrays imagePoints1, cv::InputArrayOfArrays imagePoints2,
    c_stereo_camera_intrinsics & intrinsics,
    c_stereo_camera_extrinsics & extrinsics,
    int flags,
    const cv::TermCriteria & term,
    /* out, opt */ cv::Matx33d * E,
    /* out, opt */ cv::Matx33d * F,
    /* out, opt */ std::vector<cv::Vec3d> * rvecs,
    /* out, opt */ std::vector<cv::Vec3d> * tvecs,
    /* out, opt */ cv::Mat1d * perViewErrors)
{
  double rmse = -1;

  try {

    rmse =
        cv::stereoCalibrate(objectPoints,
            imagePoints1, imagePoints2,
            intrinsics.camera[0].camera_matrix, intrinsics.camera[0].dist_coeffs,
            intrinsics.camera[1].camera_matrix, intrinsics.camera[1].dist_coeffs,
            intrinsics.camera[0].image_size,
            extrinsics.R, extrinsics.T,
            E ? *E : cv::noArray(),
            F ? * F : cv::noArray(),
            rvecs ? *rvecs : cv::noArray(),
            tvecs ? *tvecs : cv::noArray(),
            perViewErrors ? * perViewErrors : cv::noArray(),
            flags,
            term);
  }
  CATCH("from cv::stereoCalibrate()");

  return rmse;
}



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
    /* out, opt*/ cv::Mat2f rmaps[2],
    /* out, opt*/ c_stereo_camera_intrinsics * new_intrinsics,
    /* out, opt*/ c_stereo_camera_extrinsics * new_extrinsics,
    /* out, opt*/ cv::Matx33d outputR[2],
    /* out, opt*/ cv::Matx34d outputP[2],
    /* out, opt*/ cv::Matx44d * outputQ,
    /* out, opt*/ cv::Rect outputValidRoi[2])
{
  INSTRUMENT_REGION("");

  if ( intrinsics.camera[0].image_size != intrinsics.camera[1].image_size ) {
    CF_ERROR("ERROR: Sorry, that will not work: left and right camera images are not equal");
    return false;
  }

  const cv::Size & srcImageSize =
      intrinsics.camera[0].image_size;

  if ( srcImageSize.empty() ) {
    CF_ERROR("Invalid argument: source image size is not specified in stereo camera intrinsics");
    return false;
  }


  cv::Matx33d R[2];
  cv::Matx34d P[2]; // 'rectified' camera matrices
  cv::Matx44d Q;
  cv::Rect validRoi[2];


  cv::stereoRectify(intrinsics.camera[0].camera_matrix, intrinsics.camera[0].dist_coeffs,
      intrinsics.camera[1].camera_matrix, intrinsics.camera[1].dist_coeffs,
      srcImageSize, extrinsics.R, extrinsics.T,
      R[0], R[1], P[0], P[1], Q,
      cv::CALIB_ZERO_DISPARITY,
      alpha,
      dstImageSize,
      &validRoi[0],
      &validRoi[1]);

  // some temporary debug stuff
  //  pmat( euler_angles(R[0]) * 180 / CV_PI, "R1");
  //  pmat( euler_angles(R[1]) * 180 / CV_PI, "R2");
  //  CF_DEBUG("validRoi[0]={%d %d %dx%d}", validRoi[0].x, validRoi[0].y, validRoi[0].width, validRoi[0].height);
  //  CF_DEBUG("validRoi[1]={%d %d %dx%d}", validRoi[1].x, validRoi[1].y, validRoi[1].width, validRoi[1].height);


  if ( rmaps ) {

    for ( int i = 0; i < 2; ++i ) {
      cv::initUndistortRectifyMap(intrinsics.camera[i].camera_matrix,
          intrinsics.camera[i].dist_coeffs,
          R[i], P[i],
          dstImageSize,
          rmaps[i].type(),
          rmaps[i],
          cv::noArray());
    }
  }

  if ( new_intrinsics ) {

    for ( int i = 0; i < 2; ++i ) {

      new_intrinsics->camera[i].dist_coeffs.clear();

      new_intrinsics->camera[i].image_size =
          dstImageSize;

      new_intrinsics->camera[i].camera_matrix =
          camera_matrix_from_stereo_projection_matrix(P[i]);
    }
  }

  if ( new_extrinsics ) {

    new_extrinsics->R =
        cv::Matx33d::eye();

    new_extrinsics->T =
        cv::Vec3d(-1. / cv::norm(extrinsics.T), 0, 0);
  }

  if ( outputR ) {
    outputR[0] = R[0];
    outputR[1] = R[1];
  }

  if ( outputP ) {
    outputP[0] = P[0];
    outputP[1] = P[1];
  }

  if ( outputQ ) {
    *outputQ = Q;
  }

  if ( outputValidRoi ) {
    outputValidRoi[0] = validRoi[0];
    outputValidRoi[1] = validRoi[1];
  }

  return true;
}




