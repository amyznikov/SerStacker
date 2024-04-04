/*
 * c_hdl_range_image_config_routine.cc
 *
 *  Created on: Feb 28, 2024
 *      Author: amyznikov
 */

#include "c_hdl_range_image_config_routine.h"

void c_hdl_range_image_config_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_CTRL(ctls, azimuthal_resolution, "azimuthal_resolution", "Range image azimuthal resolution in [deg/pix]");
  BIND_CTRL(ctls, start_azimuth, "start_azimuth", "Range image start azimuth in [deg]");
}

bool c_hdl_range_image_config_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, azimuthal_resolution);
    SERIALIZE_PROPERTY(settings, save, *this, start_azimuth);
    return true;
  }
  return false;
}

bool c_hdl_range_image_config_routine::process(c_hdl_data_frame * hdl)
{
  c_hdl_range_image & range_image =
      hdl->range_image();

  range_image.set_azimuthal_resolution(azimuthal_resolution_ * CV_PI / 180);
  range_image.set_start_azimuth(start_azimuth_ * CV_PI / 180);

  return true;
}
