/*
 * c_homography_test_routine.cc
 *
 *  Created on: Mar 6, 2023
 *      Author: amyznikov
 */

#include "c_homography_test_routine.h"
#include <core/proc/camera_calibration/camera_pose.h>


static cv::Mat2f create_perspective_remap(const cv::Size & output_size,
    const cv::Matx33f & input_camera_matrix,
    const cv::Matx33f & output_camera_matrix,
    const cv::Vec3f & A,
    const cv::Vec3f & T)
{

  const cv::Matx33f R =
      build_rotation(A);

  const cv::Matx33f output_camera_matrix_inv =
      output_camera_matrix.inv();

  cv::Mat2f m(output_size);

  cv::Vec3f T0 =
      R * cv::Vec3f(0, 0, 1);

  T0[2] -= 1;

  for ( int y = 0; y < output_size.height; ++y ) {
    for ( int x = 0; x < output_size.width; ++x ) {

      const cv::Vec3f pos =
          input_camera_matrix * (R * output_camera_matrix_inv * cv::Vec3f(x, y, 1) - T0 + cv::Vec3f(0, 0, T[2]));

      m[y][x][0] = pos[0] / pos[2] + T[0];
      m[y][x][1] = pos[1] / pos[2] + T[1];
    }
  }

  return m;

}

void c_homography_test_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, rotation, "rotation angles in [deg]");
  BIND_PCTRL(ctls, translation, "translation vector");
  BIND_PCTRL(ctls, focus, "focus distance in pixels");
  BIND_PCTRL(ctls, output_size, "output image size");

}

bool c_homography_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, rotation);
    SERIALIZE_PROPERTY(settings, save, *this, translation);
    SERIALIZE_PROPERTY(settings, save, *this, focus);
    SERIALIZE_PROPERTY(settings, save, *this, output_size);
    return true;
  }
  return false;
}

bool c_homography_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  ////////////
//  const cv::Size input_size =
//      image.size();
//
//  const cv::Matx33f K(
//      F, 0, input_size.width / 2.,
//      0, F, input_size.height / 2.,
//      0, 0, 1
//      );
//
//  const cv::Matx33f Kinv =
//      K.inv();
//
//
//  const cv::Matx43f I(
//      1, 0, 0,
//      0, 1, 0,
//      0, 0, 1,
//      0, 0, 1);
//
//  const cv::Matx33f R =
//      build_rotation(A * CV_PI / 180);
//
//  const cv::Matx34f RT(
//      R(0,0), R(0,1), R(0,2), T(0),
//      R(1,0), R(1,1), R(1,2), T(1),
//      R(2,0), R(2,1), R(2,2), T(2));
//
//  cv::Matx33f HH =
//      K * RT * I * Kinv;
//
//  cv::warpPerspective(image.getMat(), image, HH, input_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT);


  ////////////


  const cv::Size input_size =
      image.size();

  const cv::Size output_size =
      output_size_.empty() ? input_size : output_size_;

  const cv::Matx33f input_camera_matrix(
      F, 0, input_size.width / 2.,
      0, F, input_size.height / 2.,
      0, 0, 1
      );

  const cv::Matx33f output_camera_matrix(
      F, 0, output_size.width / 2.,
      0, F, output_size.height / 2.,
      0, 0, 1
      );

  const cv::Mat2f H =
      create_perspective_remap(output_size,
          input_camera_matrix,
          output_camera_matrix,
          A * CV_PI / 180,
          T);

  cv::remap(image.getMat(), image, H, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT,
      cv::Scalar::all(0));

  if( mask.empty() ) {

    cv::remap(cv::Mat1b(image.size(), (uint8_t) 255), mask, H, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(0));

  }
  else {

    cv::remap(mask.getMat(), mask, H, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(0));

    cv::warpPerspective(mask, mask,
        F,
        image.size(),
        cv::INTER_LINEAR, // Note that cv::INTER_LINEAR can introduce some blur on kitti images
        cv::BORDER_CONSTANT);

  }

  cv::compare(mask, 252, mask, cv::CMP_GE);

  return true;
}
