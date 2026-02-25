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
    _azimuthal_resolution = v;
  }

  double azimuthal_resolution() const
  {
    return _azimuthal_resolution;
  }

  void set_start_azimuth(double v)
  {
    _start_azimuth = v;
  }

  double start_azimuth() const
  {
    return _start_azimuth;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(c_hdl_data_frame * hdl) final;

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
  {
    ctlbind(ctls, "azimuthal_resolution", ctx(&this_class::_azimuthal_resolution), "Range image azimuthal resolution in [deg/pix]");
    ctlbind(ctls, "start_azimuth", ctx(&this_class::_start_azimuth), "Range image start azimuth in [deg]");
  }

protected:
  double _azimuthal_resolution = c_hdl_range_image::default_azimuthal_resolution() * 180 / CV_PI; // deg
  double _start_azimuth = c_hdl_range_image::default_start_azimuth() * 180 / CV_PI; // deg
};

#endif /* __c_hdl_range_image_config_routine_h__ */
