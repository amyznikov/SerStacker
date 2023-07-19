/*
 * birds-eye-view.cc
 *
 *  Created on: Mar 28, 2021
 *      Author: amyznikov
 */
#include "birds-eye-view.h"
#include <core/proc/pose.h>
#include <core/debug.h>

// prevent conflicts with msvs min() and max() macroses
#ifdef _WIN32
# ifdef min
#   undef min
# endif
# ifdef max
#  undef max
# endif
#endif

// Uncomment this when debugging
//namespace {
//
//void pmat(const cv::Matx33d & F, const std::string & name)
//{
//  fprintf(stderr, "%s: {\n"
//      "%+12.6f\t%+12.6f\t%+12.6f\n"
//      "%+12.6f\t%+12.6f\t%+12.6f\n"
//      "%+12.6f\t%+12.6f\t%+12.6f\n"
//      "}\n",
//      name.c_str(),
//      F(0, 0), F(0, 1), F(0, 2),
//      F(1, 0), F(1, 1), F(1, 2),
//      F(2, 0), F(2, 1), F(2, 2));
//
//}
//
//void pmat(const cv::Vec3d & T, const std::string & name)
//{
//  fprintf(stderr, "%s: {\n"
//      "%+12.6f\n"
//      "%+12.6f\n"
//      "%+12.6f\n"
//      "}\n",
//      name.c_str(),
//      T(0),
//      T(1),
//      T(2));
//
//}
//}

void suggestBirdsEyeViewRoadRange(const cv::Matx34d & ref2CamMatrix,
    double road_min_distance_ahead, double road_max_distance_ahead,
    double road_space_left, double road_space_right,
    double * xb, double * xt,
    double * yl, double * yr)
{
  // Decompose Re2Cam matrix into rtotation and translation
  cv::Matx33d R;
  cv::Vec3d T;

  split_pose(ref2CamMatrix, R, T);

  // Get camera position in Ref (ISO 8855 VS) coordinate system (field 'position' in file sensor_setup.yaml)
  const cv::Vec3d cpos = -R.t() * T;

  // Swap axes from VS to Camera system for more comfortable work with euler angles.
  static const cv::Matx33d S =
      cv::Matx33d(
          0, -1, 0,
          0, 0, -1,
          1, 0, 0).t();

  R = S * R;

  // Compute euler angles, actualy only yaw angle has matter
  const cv::Vec3d E =
      euler_angles(R);

  // Determine the view direction (yaw angle) of the camera.
  // Yaw angle grows clockwise from 0 to 360 deg, startimg from vechicle frontal direction.
  double yaw = E(2) * 180 / CV_PI;
  if( yaw < 0 && (yaw += 360) >= 360 ) {
    yaw = 0;
  }

  enum VIEW_DIRECTION
  {
    VIEW_DIRECTION_0,
    VIEW_DIRECTION_45,
    VIEW_DIRECTION_90,
    VIEW_DIRECTION_135,
    VIEW_DIRECTION_180,
    VIEW_DIRECTION_225,
    VIEW_DIRECTION_270,
    VIEW_DIRECTION_315,
  } view_direction = VIEW_DIRECTION_0;

  if( yaw <= 45. / 2 ) {
    view_direction = VIEW_DIRECTION_0;
  }
  else if( yaw >= 45 - 45. / 2 && yaw <= 45 + 45. / 2 ) {
    view_direction = VIEW_DIRECTION_45;
  }
  else if( yaw >= 90 - 45. / 2 && yaw <= 90 + 45. / 2 ) {
    view_direction = VIEW_DIRECTION_90;
  }
  else if( yaw >= 135 - 45. / 2 && yaw <= 135 + 45. / 2 ) {
    view_direction = VIEW_DIRECTION_135;
  }
  else if( yaw >= 180 - 45. / 2 && yaw <= 180 + 45. / 2 ) {
    view_direction = VIEW_DIRECTION_180;
  }
  else if( yaw >= 225 - 45. / 2 && yaw <= 225 + 45. / 2 ) {
    view_direction = VIEW_DIRECTION_225;
  }
  else if( yaw >= 270 - 45. / 2 && yaw <= 270 + 45. / 2 ) {
    view_direction = VIEW_DIRECTION_270;
  }
  else if( yaw >= 315 - 45. / 2 && yaw <= 315 + 45. / 2 ) {
    view_direction = VIEW_DIRECTION_315;
  }
  else {
    view_direction = VIEW_DIRECTION_0;
  }

  // Suggest recommended road range depending on camera view direction
  // CF_DEBUG("Yaw=%g deg VIEW_DIRECTION=%d", yaw, view_direction);

  switch (view_direction) {
    case VIEW_DIRECTION_0:
      *xb = +road_min_distance_ahead;
      *xt = +road_max_distance_ahead;
      *yl = +road_space_left;
      *yr = -road_space_right;
      break;

    case VIEW_DIRECTION_45:
      *xb = +road_min_distance_ahead;
      *xt = +road_max_distance_ahead;
      *yl = std::min(0.0, +road_space_left);
      *yr = -road_space_right;
      break;

    case VIEW_DIRECTION_90:
      *xb = -road_max_distance_ahead / 2;
      *xt = +road_max_distance_ahead / 2;
      *yl = std::max(0.0, -road_space_left);
      *yr = -road_space_right;
      break;

    case VIEW_DIRECTION_135:
      *xb = -road_min_distance_ahead;
      *xt = -road_max_distance_ahead;
      *yl = -road_space_left;
      *yr = std::min(0., +road_space_right);
      break;

    case VIEW_DIRECTION_180:
      *xb = cpos(0) - road_min_distance_ahead;
      *xt = -road_max_distance_ahead;
      *yl = -road_space_left;
      *yr = +road_space_right;
      break;

    case VIEW_DIRECTION_225:
      *xb = cpos(0) - road_min_distance_ahead;
      *xt = -road_max_distance_ahead;
      *yl = std::max(std::max(1.0, cpos(1)), -road_space_left);
      *yr = +road_space_right;
      break;

    case VIEW_DIRECTION_270:
      *xb = -road_max_distance_ahead / 2;
      *xt = +road_max_distance_ahead / 2;
      *yl = road_space_left;
      *yr = std::max(0., -road_space_right);
      break;

    case VIEW_DIRECTION_315:
      *xb = +road_min_distance_ahead;
      *xt = +road_max_distance_ahead;
      *yl = +road_space_left;
      *yr = std::max(0.0, -road_space_right);
      break;
  }
}

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
    int ddepth)
{

  // Camera matrix
  const cv::Matx33d K =
      cameraMatrix.getMat();

  // ref (vechicle) -> cam  R|T matrix
  cv::Matx34d RT =
      ref2CamExtrinsicMatrix.getMat();

  // ref -> cam rotation
  cv::Matx33d R;

  // ref -> cam translation
  cv::Vec3d T;

  // split ref->cam 3x4 matrix into rotation and translation
  split_pose(RT, R, T);

  if( reference_pitch || reference_yaw || reference_roll ) {

    // Update rotation matrix for road->ref correction for oxts snaking.
    // Still not sure if order and signs are correct for zenseact oxts, need to be checked.
    cv::Matx33d Rroll, Rpitch, Ryaw;

    build_rotation(reference_roll, reference_pitch, reference_yaw,
        &Rroll, &Rpitch, &Ryaw);

    R = R * Rroll * Rpitch * Ryaw;
  }

  // Output image corners corresponding to road_points_in_vechicle_coordinates
  const cv::Point2f output_image_corners[4] = {
      cv::Point2f(0, outImageSize.height - 1), // BL
      cv::Point2f(outImageSize.width - 1, outImageSize.height - 1), // BR
      cv::Point2f(outImageSize.width - 1, 0), // TR
      cv::Point2f(0, 0), // TL
      };

  // Requested Road points in ISO 8855 vechicle coordinate system:
  //  x-> forward, y-> left, z-> up
  const cv::Vec3d road_points_in_vechicle_coordinates[4] = {
      cv::Vec3d(road_xb, road_yl, -reference_height), // BL
      cv::Vec3d(road_xb, road_yr, -reference_height), // BR
      cv::Vec3d(road_xt, road_yr, -reference_height), // TR
      cv::Vec3d(road_xt, road_yl, -reference_height), // TL
      };

  // Computed road poits in homogenous camera coordinates
  cv::Point2f road_points_in_image_coordinates[4];

  // Step 1
  //  Convert each requested road point and corresponding output image corner into
  //  image (pixel) coordinates.

  for( uint i = 0; i < 4; ++i ) {

    // Convert road point from vechicle (reference) coordinates into image (pixels) coordinate system
    const cv::Vec3d road_point_in_image_coordinates =
        K * (R * road_points_in_vechicle_coordinates[i] + T);

    road_points_in_image_coordinates[i].x =
        road_point_in_image_coordinates[0] / road_point_in_image_coordinates[2];

    road_points_in_image_coordinates[i].y =
        road_point_in_image_coordinates[1] / road_point_in_image_coordinates[2];
  }

  // Step 2
  // Compute planar homography form 4 pxel matches of road poits and corresponding output image corners.

  const cv::Mat H =
      cv::getPerspectiveTransform(output_image_corners,
          road_points_in_image_coordinates);

  if( H.empty() ) {
    CF_ERROR("getPerspectiveTransform() fails");
    return false;
  }

  // Step 3
  // Convert to requested depth and return result.

  if( outputHomography.fixedType() ) {
    ddepth = outputHomography.depth();
  }
  else if( ddepth != CV_32F && ddepth != CV_64F ) {
    ddepth = H.depth();
  }

  cv::Mat(H).convertTo(
      outputHomography, ddepth);

  return true;
}

cv::Size estimateBirdsEyeViewImageSize(double horizontal_resolution, double vertical_resolution,
    double road_xb, double road_xt, double road_yl, double road_yr)
{
  if( !(horizontal_resolution > 0) ) {
    horizontal_resolution = 0.025;
  }

  if( !(vertical_resolution > 0) ) {
    vertical_resolution = 0.050;
  }

  const double x_road_extent = std::abs(road_xt - road_xb);
  const double y_road_extent = std::abs(road_yl - road_yr);

  const int width = (((int) (y_road_extent / horizontal_resolution) + 1) & ~0x1);
  const int height = (((int) (x_road_extent / vertical_resolution) + 1) & ~0x1);

  return cv::Size(width, height);
}

template<class T>
static inline void composeSimpleRef2CamMatrix_(cv::Matx<T, 3, 4> & dst,
    double roll, double pitch, double yaw,
    double x, double y, double z)
{
  cv::Matx33d Rx, Ry, Rz;

  // Swap axes from VS to Camera system
  static const cv::Matx33d S(
      0, -1, 0,
      0, 0, -1,
      1, 0, 0);

  build_rotation(roll, pitch, yaw,
      &Rx, &Ry, &Rz);

  dst = build_pose(S * Rz * Ry * Rx,
      cv::Vec3d(x, y, z));
}

void composeSimpleRef2CamMatrix(cv::Matx34d & dst,
    double roll, double pitch, double yaw,
    double x, double y, double z)
{
  composeSimpleRef2CamMatrix_(dst, roll, pitch, yaw, x, y, z);
}

void composeSimpleRef2CamMatrix(cv::Matx34f & dst,
    double roll, double pitch, double yaw,
    double x, double y, double z)
{
  composeSimpleRef2CamMatrix_(dst, roll, pitch, yaw, x, y, z);
}
