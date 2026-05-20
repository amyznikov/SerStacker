/*
 * c_draw_saturn_ellipse_routine.cc
 *
 *  Created on: Jul 10, 2024
 *      Author: amyznikov
 */

#include "c_draw_saturn_ellipse_routine.h"
//#include <core/proc/ellipsoid.h>
//#include <core/proc/pose.h>


void c_draw_saturn_ellipse_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "auto_location", ctx, &this_class::auto_location, &this_class::set_auto_location, "");
  ctlbind(ctls, "gbsigma", ctx, &this_class::gbsigma, &this_class::set_gbsigma, "");
  ctlbind(ctls, "stdev_factor", ctx, &this_class::stdev_factor, &this_class::set_stdev_factor, "");
  ctlbind(ctls, "se_close_radius", ctx, &this_class::se_close_radius, &this_class::set_se_close_radius, "");

  ctlbind(ctls, "equatorial_radius", ctx, &this_class::equatorial_radius, &this_class::set_equatorial_radius, "");
  ctlbind(ctls, "ring_radius", ctx, &this_class::ring_radius, &this_class::set_ring_radius, "");
  ctlbind(ctls, "center", ctx, &this_class::center, &this_class::set_center, "");
  ctlbind(ctls, "pose", ctx, &this_class::pose, &this_class::set_pose,
      "Ellipsoid pose rotation angles Ax;Ay;Az in [deg]");
  ctlbind(ctls, "zrotation_remap", ctx, &this_class::zrotation_remap, &this_class::set_zrotation_remap,
      "derotation time in [sec]");

  ctlbind(ctls, "longitude_step", ctx, &this_class::longitude_step, &this_class::set_longitude_step, "");
  ctlbind(ctls, "latidute_step", ctx, &this_class::latidute_step, &this_class::set_latidute_step, "");
  ctlbind(ctls, "outline_color", ctx, &this_class::outline_color, &this_class::set_outline_color, "");
  ctlbind(ctls, "lines_color", ctx, &this_class::lines_color, &this_class::set_lines_color, "");

  ctlbind(ctls, "show_ring", ctx, &this_class::show_ring, &this_class::set_show_ring, "");
  ctlbind(ctls, "show_smask", ctx, &this_class::show_smask, &this_class::set_show_smask, "");
  ctlbind(ctls, "show_sbox", ctx, &this_class::show_sbox, &this_class::set_show_sbox, "");

  ctlbind(ctls, "", ctx, &this_class::print_debug_info, &this_class::set_print_debug_info,
      "print detected ellipse parameters into debug log");
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
