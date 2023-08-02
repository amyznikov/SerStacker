/*
 * camera_settings.cc
 *
 *  Created on: Aug 1, 2023
 *      Author: amyznikov
 */

#include "camera_settings.h"
#include <core/debug.h>

bool load_settings(c_config_setting settings, c_camera_intrinsics * c)
{
  if( !load_settings(settings, "image_size", &c->image_size) ) {
    CF_ERROR("load_settings(image_size) fails for c_camera_intrinsics");
    return false;
  }

  if( !load_settings(settings, "camera_matrix", &c->camera_matrix) ) {
    CF_ERROR("load_settings(camera_matrix) fails for c_camera_intrinsics");
    return false;
  }

  if( settings["dist_coeffs"] && !load_settings(settings, "dist_coeffs", &c->dist_coeffs) ) {
    CF_ERROR("load_settings(image_size) fails for c_camera_intrinsics");
    return false;
  }

  return true;
}

bool save_settings(c_config_setting settings, const c_camera_intrinsics & c)
{
  save_settings(settings, "image_size", c.image_size);
  save_settings(settings, "camera_matrix", c.camera_matrix);
  save_settings(settings, "dist_coeffs", c.dist_coeffs);
  return true;
}
