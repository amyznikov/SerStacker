/*
 * birds-eye-view.h
 *
 *  Created on: Mar 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __birds_eye_view_h__
#define __birds_eye_view_h__

#include <opencv2/opencv.hpp>

/**
 @brief Create bird's-eye view homography matrix for inverse perspective mapping.

This function tries to mimic the Matlab's birdsEyeView() @ref <https://www.mathworks.com/help/driving/ref/birdseyeview.html>.

Assumed axes orientations are:

ISO 8855 Vechicle System (VS):
  
  x -> forward, y -> left, z -> up.
  
  Roll is rotation around of x, Pitch is around of y,  Yaw is around of z.

Camera system (Cam):

  x -> right, y -> bottom, z -> forward.

@image html Vehicle-Axis-System-ISO-8855-2011.png "Vehicle-Axis-System-ISO-8855-2011"

The road is treated in ISO 8855 Vechicle System, therefore road X coordinate is directed forward,
road Y coordinate is directed left, road Z coordinate is directed up.

Be carefull when working with side or backward looking cameras: the road still treated in vechicle VS,
therefore backward road X coordinates are negative.

@image html birds-eye-view-road.png "birds-eye-view-road"




@param cameraMatrix
     A 3x3 CV_32F or CV_64F camera intrinsics matrix, can be cv::Matx33f or cv::Matx33d.

@param ref2CamExtrinsicMatrix
      A 3x4 CV_32F or CV_64F  Ref->Cam R|T extrinsics matrix.
      You can get it from @ref load_camera_calibration_from_sensor_setup_yml(), or setup manually using @ref composeSimpleRef2CamMatrix().
      The geometrical sense of extrinsics is intentionally made compatible with sensor_setup.yml where conversion
      from Reference (VS) to Camera system imply rotation around of VS axes first, followed by translation in
      Camera system: \f$P_{cam} = R_{vs} P_{vs} + T_{cam}\f$.
      Therefore the translation vector \f$T_{cam}\f$ is applied after the rotation \f$R_{vs}\f$, e.g. actually in camera axes.

@param reference_roll
    Optional correction for oxts snaking in roll (around of looking forward X axis of VS), [radians]

@param reference_pitch
    Optional correction for oxts snaking in pitch (around of looking left Y axis of VS), [radians]

@param reference_yaw
    Optional correction for oxts snaking in yaw (around of looking up Z axis of VS), [radians]

@param reference_height
    The height of reference coordinate system (VS) above the road, in meters, [m].
    I am still not sure if I understand sensor_setup.yml completelly correctly, but it seems that the
    zero point of zenseact reference system reside somewhere in the IMU box near to rear bridge.
    GT team set this parameter to 0.35m in GroundTruthTools/LIDAR_PostProcess/+lanes/@LaneMarkers/processImage.m
    arguing that "Distance to ground should be based on characters calibration, but they are lacking distance
    to ground right now. Add height over rear-axls with 35cm to compensate for wheel size".

@param road_xmin
    The nearest X coordinate of the road region to transform, [m].
    This maps to bottom side of output image.
    For forward-lookig camera default is 5m.
    Adjust manually for your camera, hood length, and looking direction.
    Note that for backward-looking camera this probably is negative value, for example -5m.

@param road_xmax
    The maximal X coordinate of the road region to transform, [m].
    This maps to top side of output image.
    For forward-lookig camera default is 100 m.
    Adjust manually for your camera and looking direction.
    Note that for backward-looking camera this probably is negative value, for example -100m.

@param road_ymin
    The mimimal Y coordinate of the road region to transform, [m].
    This maps to right side of output image.
    For forward-lookig camera on right-hand traffic the default is -6 m.
    Adjust manually for your road width.

@param road_ymax
    The maximal Y coordinate of the road region to transform, [m].
    This maps to left side of output image.
    For forward-lookig camera on right-hand traffic the default is +10 m.
    Adjust manually for your road width.

@param outImageSize
    Similar to the Matlab's equivalent, defines the size, in pixels, of the output bird's-eye-view image.
    You can use the @ref adjustBirdsEyeViewImageSize() to automate the estimation of outputImageSize to mantain
    required aspect ratio based on road ROI.

@param outputHomography
    Computed planar 3x3 homography matrix of CV_32FC1 or CV_64FC1 type.
    For non-fixed-type outputs the required type can be specified explicitly using @ref ddepth parameter.

@param ddepth
    Optional required output matrix depth, can be CV_32F or CV_64F.


Example:

@code
  #include <glv/core/road_detection/birds-eye-view.h>
  #include <glv/core/dataset/configurable/loaders/sensor_setup_yml.h>

  // ...

  // 1. Load required camera intrinsics and extrinsics from sensor_setup.yml

  c_camera_projection_intrinsics intrinsics;
  cv::Matx34d ref2CamMatrix;

  load_camera_calibration_from_sensor_setup_yml("sensor_setup.yml",
      "FC",
      &ref2CamMatrix,
      nullptr,
      &intrinsics,
      nullptr);


  // 2. Convert sensor setup intrinsics to 3x3 camera matrix

  cv::Matx33d cameraMatrix;

  projectionIntrinsics2CameraMaxtrix(intrinsics,
      &cameraMatrix);


  // 3 Compute planar 3x3 homography matrix for inverse perspective mapping

  // Select road region to be projected, in meters:
  double xb, xt, yl, yr;

  suggestBirdsEyeViewRoadRange(ref2CamMatrix,
      min_distance_ahead, max_distance_ahead,
      road_space_left, road_space_right,
      &xb, &xt,
      &yl, &yr);

  // adjust output image size
  const cv::Size outputImageSize = estimateBirdsEyeViewImageSize(
      detector.birds_eye_view_horizontal_resolution(),
      detector.birds_eye_view_vertical_resolution(),
      xb, xt, yl, yr);

  // Computed planar homography matrix
  cv::Matx33f H;


  createBirdsEyeViewHomography(cameraMatrix,
      ref2CamMatrix,
      0,  // no oxts correction in roll
      0,  // no oxts correction in pitch
      0,  // no oxts correction in yaw
      reference_height = 0.35, // height of reference VS (oxts?) reference point above the ground
      road_xb, road_xt, // road region to be mapped
      road_yl, road_yr, // road region to be mapped
      outputImageSize, // The size of output birds-eye-view image in pixels
      H);


  // 4 Apply computed planar homography matrix to input image

  cv::Mat input_image, output_image;

  load_image(&input_image, "input_image.png");

  cv::warpPerspective(input_image, output_image, H, outputImageSize,
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

  save_image(output_image, "output_image.png");

@endcode

  Complete working example can be found in apps/monocamera-birds-eye-view-example directory.

**/
bool createBirdsEyeViewHomography(cv::InputArray cameraMatrix,
    cv::InputArray ref2CamExtrinsicMatrix,
    double reference_roll,
    double reference_pitch,
    double reference_yaw,
    double reference_height,
    double road_xb, double road_xt,
    double road_yl, double road_yr,
    const cv::Size & outImageSize,
    cv::OutputArray outputHomography,
    int ddepth = -1);

/**
@brief Utility routine to compute output birds eye view image size, in pixels, for BirdsEyeView homography inverse perspective mapping.

This procedure computes output bird's-eye-view image size based on desired image resoltion in meters per pixel and
provided road range in meters.

@param horizontal_resolution_in_meters_per_pixel
    User-specified desired resolution in meters per pixel for horizontal extent (width) of the birds-eye-vew image.

@param vertical_resolution_in_meters_per_pixel
    User-specified desired resolution in meters per pixel for horizontal extent (width) of the birds-eye-vew image.

@param xb
    The nearest X coordinate of the road region to transform, [m].
    This maps to bottom side of output image.
    For forward-lookig camera default is 5m.
    Adjust manually for your camera, hood length, and looking direction.
    Note that for backward-looking camera this probably is negative value, for example -5m.

@param xt
    The maximal X coordinate of the road region to transform, [m].
    This maps to top side of output image.
    For forward-lookig camera default is 35 m.
    Adjust manually for your camera and looking direction.
    Note that for backward-looking camera this probably is negative value, for example -35m.

@param yl
    The maximal Y coordinate of the road region to transform, [m].
    This maps to left side of output image.
    For forward-lookig camera on right-hand traffic the default is +8 m.
    Adjust manually for your road width.

@param yr
    The mimimal Y coordinate of the road region to transform, [m].
    This maps to right side of output image.
    For forward-lookig camera on right-hand traffic the default is -8 m.
    Adjust manually for your road width.


For convenience the desired image resoltion shoud be requested from actual line detector,
and the road extent can be siuggested by @ref suggestBirdsEyeViewRoadRange() :
@code

@endcode

  double xb, xt, yl, yr;

  // Suggested appropriate road range basing on camera extrinsics and desired road distances
  suggestBirdsEyeViewRoadRange(args.ref2CamMatrix,
      args.min_distance_ahead, args.max_distance_ahead,
      args.road_space_left, args.road_space_right,
      &xb, &xt,
      &yl, &yr);


  // Suggested approprate birds-eye-view image size
  const cv::Size outSize = estimateBirdsEyeViewImageSize(
      detector.birds_eye_view_horizontal_resolution(),
      detector.birds_eye_view_vertical_resolution(),
      xb, xt, yl, yr);


  // Compute homographt matrix
  fOK = createBirdsEyeViewHomography(args.cameraMatrix,
      args.ref2CamMatrix,
      args.extrinsics_correction_in_roll,
      args.extrinsics_correction_in_pitch,
      args.extrinsics_correction_in_yaw,
      args.reference_height,
      xb, xt,
      yl, yr,
      outSize,
      H,
     -1);

  // Convert image to birds-eye-view
  cv::warpPerspective(args.input_image, birdsEyeViewImage, H, outSize,
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT);

**/

cv::Size estimateBirdsEyeViewImageSize(double horizontal_resolution_in_meters_per_pixel, double vertical_resolution_in_meters_per_pixel,
    double xb, double xt, double yl, double yr);


/**
@brief Utility routine to compose simple mapping from ISO 8855 Vechicle system  (reference zenseact system)
into camera coordinate system.

This function can be usefull when no sensor_setup_yml is available and Ref->Cam mapping need to be set manually.

This function is created mainly as helper to @ref createBirdsEyeViewHomography() when no sensor setup yml available.


Assumed axes orientation:

ISO 8855 Vechicle System (VS):
  x -> forward, y -> left, z -> up.
  Roll is rotation around of x, Pitch is around of y,  Yaw is around of z.

Camera system (Cam):
  x -> right, y -> bottom, z -> forward.


The gemetrical sense of parameters is intentionally made compatible with sensor_setup.yml where conversion
from Ref (VS) to Cam imply rotation around of VS axes first, followed by translation in Camera system: \f$P_{cam} = R_{vs} P_{vs} + T_{cam}\f$.

Therefore the translation vector \f$T_{cam}\f$ is applied after the rotation \f$R_{vs}\f$, e.g. actually in camera axes.

@param dst
    Output 3x4 R|T destination matrix.

@param roll
    Rotation around of X (forward looking) Vechicle Axis, [radians].

@param pitch
    Rotation around of Y (left looking) Vechicle Axis, [radians].

@param yaw
    Rotation around of Z (up looking) Vechicle Axis, [radians].

@param tx
    Translation along X (right looking) Camera Axis, [m]

@param ty
    Translation along Y (down looking) Camera Axis, [m]

@param tz
    Translation along Z (forwad looking) Camera Axis, [m]

Example:

@code
  cv::Matx33d Ref2Cam;

  composeSimpleRef2CamMatrix(
      Ref2Cam, // reference to destination
      0,                  // roll in VS
      -2.5 * CV_PI / 180, // pitch in VS
      0,                  // yaw in VS
      0,                  // Tx in camera
      0,                  // Ty in camera
      5                   // Tz in camera
      );

@endcode
 **/
void composeSimpleRef2CamMatrix(cv::Matx34d & dst,
    double roll, double pitch, double yaw,
    double tx, double ty, double tz);

/**
 * @overload
 **/
void composeSimpleRef2CamMatrix(cv::Matx34f & dst,
    double roll, double pitch, double yaw,
    double x, double y, double z);

/**@brief Suggest recommended road range depending on the position and orientation of the camera.
 *
 * @param [in] ref2CamMatrix - Extrinsic matrix for current camera (Ref->Cam rotation + translation).
 * @param [in] road_min_distance_ahead - distance in meters from rear bridge (reference point) to frontal or rear bonnet.
 * @param [in] road_max_distance_ahead - max road distance ahead in meters.
 * @param [in] road_space_left - distance in meters to left edge of the road (seeing on left side of camera frame).
 * @param [in] road_space_right - distance in meters to right edge of the road (seeing on right side of camera frame).
 * @param [out] xb - suggested X road coordinate in meters to be projected onto bottom side of birds-eye-view image.
 * @param [out] xt - suggested X road coordinate in meters to be projected onto top side of birds-eye-view image.
 * @param [out] yl - suggested Y road coordinate in meters to be projected onto left side of birds-eye-view image.
 * @param [out] yr - suggested Y road coordinate in meters to be projected onto right side of birds-eye-view image.
 */
void suggestBirdsEyeViewRoadRange(const cv::Matx34d & ref2CamMatrix,
    double road_min_distance_ahead, double road_max_distance_ahead,
    double road_space_left, double road_space_right,
    double * xb, double * xt,
    double * yl, double * yr);

#endif /* __birds_eye_view_h__ */
