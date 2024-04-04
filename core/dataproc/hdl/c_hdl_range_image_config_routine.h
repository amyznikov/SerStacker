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

  void set_azimuthal_resolution(double v)
  {
    azimuthal_resolution_ = v;
  }

  double azimuthal_resolution() const
  {
    return azimuthal_resolution_;
  }

  void set_start_azimuth(double v)
  {
    start_azimuth_ = v;
  }

  double start_azimuth() const
  {
    return start_azimuth_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_hdl_data_frame * hdl) override;

protected:

  double azimuthal_resolution_ =
      c_hdl_range_image::default_azimuthal_resolution() * 180 / CV_PI; // deg

  double start_azimuth_ =
      c_hdl_range_image::default_start_azimuth() * 180 / CV_PI; // deg
};

#endif /* __c_hdl_range_image_config_routine_h__ */
