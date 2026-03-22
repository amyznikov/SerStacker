/*
 * c_hdl_range_image_config_routine.h
 *
 *  Created on: Feb 28, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_range_image_config_routine_h__
#define __c_hdl_range_image_config_routine_h__

#include "c_hdl_processor_routine.h"

class c_hdl_range_image_config_routine :
    public c_hdl_processor_routine
{
public:
  DECLARE_HDL_PROCESSOR_CLASS_FACTORY(c_hdl_range_image_config_routine,
      "hdl_range_image_config",
      "Adjust range image options");

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(c_hdl_data_frame * hdl) final;

protected:
  double _azimuthal_resolution = c_hdl_range_image::default_azimuthal_resolution() * 180 / CV_PI; // deg
  double _start_azimuth = c_hdl_range_image::default_start_azimuth() * 180 / CV_PI; // deg
};

#endif /* __c_hdl_range_image_config_routine_h__ */
