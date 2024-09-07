/*
 * c_draw_saturn_ellipse_routine.cc
 *
 *  Created on: Jul 10, 2024
 *      Author: amyznikov
 */

#include "c_draw_saturn_ellipse_routine.h"
//#include <core/proc/ellipsoid.h>
//#include <core/proc/pose.h>



void c_draw_saturn_ellipse_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, auto_location, "");
  BIND_PCTRL(ctls, gbsigma, "");
  BIND_PCTRL(ctls, stdev_factor, "");
  BIND_PCTRL(ctls, se_close_radius, "");

  BIND_PCTRL(ctls, equatorial_radius, "");
  BIND_PCTRL(ctls, ring_radius, "");
  BIND_PCTRL(ctls, center, "");
  BIND_PCTRL(ctls, pose, "Ellipsoid pose rotation angles Ax;Ay;Az in [deg]");
  BIND_PCTRL(ctls, zrotation_remap, "derotation time in [sec]");

  BIND_PCTRL(ctls, longitude_step, "");
  BIND_PCTRL(ctls, latidute_step, "");
  BIND_PCTRL(ctls, outline_color, "");
  BIND_PCTRL(ctls, lines_color, "");

  BIND_PCTRL(ctls, show_ring, "");
  BIND_PCTRL(ctls, show_smask, "");
  BIND_PCTRL(ctls, show_sbox, "");
  BIND_PCTRL(ctls, print_debug_info, "print detected ellipse parameters into debug log");


}

bool c_draw_saturn_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, auto_location);
    SERIALIZE_PROPERTY(settings, save, *this, se_close_radius);
    SERIALIZE_PROPERTY(settings, save, *this, equatorial_radius);
    SERIALIZE_PROPERTY(settings, save, *this, ring_radius);
    SERIALIZE_PROPERTY(settings, save, *this, center);
    SERIALIZE_PROPERTY(settings, save, *this, pose);
    SERIALIZE_PROPERTY(settings, save, *this, zrotation_remap);
    SERIALIZE_PROPERTY(settings, save, *this, longitude_step);
    SERIALIZE_PROPERTY(settings, save, *this, latidute_step);
    SERIALIZE_PROPERTY(settings, save, *this, outline_color);
    SERIALIZE_PROPERTY(settings, save, *this, lines_color);
    SERIALIZE_PROPERTY(settings, save, *this, show_ring);
    SERIALIZE_PROPERTY(settings, save, *this, show_smask);
    SERIALIZE_PROPERTY(settings, save, *this, show_sbox);
    SERIALIZE_PROPERTY(settings, save, *this, gbsigma);
    SERIALIZE_PROPERTY(settings, save, *this, stdev_factor);
    return true;
  }
  return false;
}

bool c_draw_saturn_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _detector.detect(image, mask) ) {
    return _detector.draw_detected(image);
  }

  return false;
}
