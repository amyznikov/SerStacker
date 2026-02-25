/*
 * c_image_undistorsion_routine.cc
 *
 *  Created on: May 31, 2024
 *      Author: amyznikov
 */

#include "c_image_undistorsion_routine.h"
#include <core/debug.h>

void c_image_undistorsion_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "dist_coeffs", ctx(&this_class::_dist_coeffs), "Array of image distortion coefficients to apply");
}

bool c_image_undistorsion_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _dist_coeffs);
    return true;
  }
  return false;
}

bool c_image_undistorsion_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  CF_ERROR("Sorry, notg implemented");

  if ( !image.empty() )  {
    // cv::undistort(image.getMat(), image, cameraMatrix, dist_coeffs_, newCameraMatrix);
  }

  return false;
}
