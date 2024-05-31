/*
 * c_image_undistorsion_routine.cc
 *
 *  Created on: May 31, 2024
 *      Author: amyznikov
 */

#include "c_image_undistorsion_routine.h"

void c_image_undistorsion_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_CTRL(ctls, dist_coeffs, "dist_coeffs", "Array of distortion coefficients");
}

bool c_image_undistorsion_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    // SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);

    return true;
  }

  return false;
}

bool c_image_undistorsion_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  if ( !image.empty() )  {

    // cv::undistort(image.getMat(), image, cameraMatrix, dist_coeffs_, newCameraMatrix);

  }

  return false;
}
