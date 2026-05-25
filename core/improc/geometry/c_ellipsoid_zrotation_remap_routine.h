/*
 * c_ellipsoid_zrotation_remap_routine.h
 *
 *  Created on: May 24, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_ellipsoid_zrotation_remap_routine_h__
#define __c_ellipsoid_zrotation_remap_routine_h__

#include <core/improc/c_image_processor.h>

class c_ellipsoid_zrotation_remap_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_ellipsoid_zrotation_remap_routine,
      "ellipsoid_zrotation_remap", "");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Point2d _center; // [pix]
  cv::Vec3d _axes;  // [pix]
  cv::Vec3d _pose; // [deg]
  double _zrotation = 0; // [deg]

  struct c_draw_ellipoid_options {
    double lat_step_deg = 30;
    double lon_step_deg = 30;
    bool enabled = true;
  } _draw_ellipoid;


  struct c_remap_options {
    bool display_counter = false;
    bool display_weights = false;
    //bool display_distance_transform = false;
    bool enabled = true;
  } _remap;
};

#endif /* __c_ellipsoid_zrotation_remap_routine_h__ */
