/*
 * c_draw_jovian_ellipse_routine.cc
 *
 *  Created on: Aug 27, 2024
 *      Author: amyznikov
 */

#include "c_draw_jovian_ellipse_routine.h"

void c_draw_jovian_ellipse_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "auto_location", ctx, &this_class::auto_location, &this_class::set_auto_location, "");
  ctlbind(ctls, "gbsigma", ctx, &this_class::gbsigma, &this_class::set_gbsigma, "");
  ctlbind(ctls, "stdev_factor", ctx, &this_class::stdev_factor, &this_class::set_stdev_factor, "");
  ctlbind(ctls, "se_close_radius", ctx, &this_class::se_close_radius, &this_class::set_se_close_radius, "");

  ctlbind(ctls, "equatorial_radius", ctx, &this_class::equatorial_radius, &this_class::set_equatorial_radius, "");
  ctlbind(ctls, "pca_blur", ctx, &this_class::pca_blur, &this_class::set_pca_blur, "");
  ctlbind(ctls, "center", ctx, &this_class::center, &this_class::set_center, "");
  ctlbind(ctls, "pose", ctx, &this_class::pose, &this_class::set_pose,
      "Ellipsoid pose rotation angles Ax;Ay;Az in [deg]");
  ctlbind(ctls, "offset", ctx, &this_class::offset, &this_class::set_offset, "");
  ctlbind(ctls, "zrotation_remap", ctx, &this_class::zrotation_remap, &this_class::set_zrotation_remap,
      "derotation time in [sec]");

  ctlbind(ctls, "longitude_step", ctx, &this_class::longitude_step, &this_class::set_longitude_step, "");
  ctlbind(ctls, "latidute_step", ctx, &this_class::latidute_step, &this_class::set_latidute_step, "");
  ctlbind(ctls, "outline_color", ctx, &this_class::outline_color, &this_class::set_outline_color, "");
  ctlbind(ctls, "lines_color", ctx, &this_class::lines_color, &this_class::set_lines_color, "");

  ctlbind(ctls, "show_bmask", ctx, &this_class::show_bmask, &this_class::set_show_bmask, "");
  ctlbind(ctls, "show_wmask", ctx, &this_class::show_wmask, &this_class::set_show_wmask, "");
  ctlbind(ctls, "show_smask", ctx, &this_class::show_smask, &this_class::set_show_smask, "");
  ctlbind(ctls, "show_sbox", ctx, &this_class::show_sbox, &this_class::set_show_sbox, "");
  ctlbind(ctls, "show_pcax", ctx, &this_class::show_pcax, &this_class::set_show_pcax, "");
  ctlbind(ctls, "show_pcay", ctx, &this_class::show_pcay, &this_class::set_show_pcay, "");

  ctlbind(ctls, "print_debug_info", ctx, &this_class::print_debug_info, &this_class::set_print_debug_info,
      "print detected ellipse parameters into debug log");
}

bool c_draw_jovian_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, auto_location);
    SERIALIZE_PROPERTY(settings, save, *this, se_close_radius);
    SERIALIZE_PROPERTY(settings, save, *this, equatorial_radius);
    SERIALIZE_PROPERTY(settings, save, *this, pca_blur);
    SERIALIZE_PROPERTY(settings, save, *this, center);
    SERIALIZE_PROPERTY(settings, save, *this, pose);
    SERIALIZE_PROPERTY(settings, save, *this, offset);
    SERIALIZE_PROPERTY(settings, save, *this, zrotation_remap);
    SERIALIZE_PROPERTY(settings, save, *this, longitude_step);
    SERIALIZE_PROPERTY(settings, save, *this, latidute_step);
    SERIALIZE_PROPERTY(settings, save, *this, outline_color);
    SERIALIZE_PROPERTY(settings, save, *this, lines_color);
    SERIALIZE_PROPERTY(settings, save, *this, show_bmask);
    SERIALIZE_PROPERTY(settings, save, *this, show_wmask);
    SERIALIZE_PROPERTY(settings, save, *this, show_smask);
    SERIALIZE_PROPERTY(settings, save, *this, show_sbox);
    SERIALIZE_PROPERTY(settings, save, *this, show_pcax);
    SERIALIZE_PROPERTY(settings, save, *this, show_pcay);
    SERIALIZE_PROPERTY(settings, save, *this, gbsigma);
    SERIALIZE_PROPERTY(settings, save, *this, stdev_factor);
    return true;
  }
  return false;
}

bool c_draw_jovian_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _derotation.detect(image, mask) ) {

    mask.release();
    _derotation.draw_detected(image);

    if ( _derotation.detector_options().draw.show_bmask ) {
      _derotation.current_bmask().copyTo(image);
    }
    else if ( _derotation.detector_options().draw.show_wmask ) {
      _derotation.current_wmask().copyTo(image);
    }
    else if ( _derotation.detector_options().draw.show_smask ) {
      _derotation.planetary_disk_ellipse_mask().copyTo(image);
    }
    else if ( _derotation.detector_options().draw.show_pcax ) {
      _derotation.pcax().copyTo(image);
    }
    else if ( _derotation.detector_options().draw.show_pcay ) {
      _derotation.pcay().copyTo(image);
    }

    return true;
  }

  CF_DEBUG("_derotation.detect(image, mask) fails");
  return false;
}
