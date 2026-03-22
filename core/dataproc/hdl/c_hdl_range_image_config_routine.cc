/*
 * c_hdl_range_image_config_routine.cc
 *
 *  Created on: Feb 28, 2024
 *      Author: amyznikov
 */

#include "c_hdl_range_image_config_routine.h"

void c_hdl_range_image_config_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "azimuthal_resolution", ctx(&this_class::_azimuthal_resolution), "Range image azimuthal resolution in [deg/pix]");
  ctlbind(ctls, "start_azimuth", ctx(&this_class::_start_azimuth), "Range image start azimuth in [deg]");
}

bool c_hdl_range_image_config_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _azimuthal_resolution);
    SERIALIZE_OPTION(settings, save, *this, _start_azimuth);
    return true;
  }
  return false;
}

bool c_hdl_range_image_config_routine::process(c_hdl_data_frame * hdl)
{
  c_hdl_range_image & range_image = hdl->range_image();

  range_image.set_azimuthal_resolution(_azimuthal_resolution * CV_PI / 180);
  range_image.set_start_azimuth(_start_azimuth * CV_PI / 180);

  return true;
}
