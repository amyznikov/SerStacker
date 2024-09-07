/*
 * c_draw_jovian_ellipse_routine.cc
 *
 *  Created on: Aug 27, 2024
 *      Author: amyznikov
 */

#include "c_draw_jovian_ellipse_routine.h"



void c_draw_jovian_ellipse_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, auto_location, "");
  BIND_PCTRL(ctls, gbsigma, "");
  BIND_PCTRL(ctls, stdev_factor, "");
  BIND_PCTRL(ctls, se_close_radius, "");

  BIND_PCTRL(ctls, equatorial_radius, "");
  BIND_PCTRL(ctls, pca_blur, "");
  BIND_PCTRL(ctls, center, "");
  BIND_PCTRL(ctls, pose, "Ellipsoid pose rotation angles Ax;Ay;Az in [deg]");
  BIND_PCTRL(ctls, offset, "");
  BIND_PCTRL(ctls, zrotation_remap, "derotation time in [sec]");

  BIND_PCTRL(ctls, longitude_step, "");
  BIND_PCTRL(ctls, latidute_step, "");
  BIND_PCTRL(ctls, outline_color, "");
  BIND_PCTRL(ctls, lines_color, "");

  BIND_PCTRL(ctls, show_bmask, "");
  BIND_PCTRL(ctls, show_wmask, "");
  BIND_PCTRL(ctls, show_smask, "");
  BIND_PCTRL(ctls, show_sbox, "");
  BIND_PCTRL(ctls, show_pcax, "");
  BIND_PCTRL(ctls, show_pcay, "");

  BIND_PCTRL(ctls, print_debug_info, "print detected ellipse parameters into debug log");


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
