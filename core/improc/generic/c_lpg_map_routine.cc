/*
 * c_lpg_map_routine.cc
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#include "c_lpg_map_routine.h"

void c_lpg_map_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_MENU_BUTTON(ctls, "Options...", "");
    BIND_MENU_COMMAND(copy_parameters_to_clipboard, "Copy parameters to clipboard");
    BIND_MENU_COMMAND(paste_parameters_from_clipboard, "Paste parameters from clipboard");
  END_MENU_BUTTON(ctls)

  BIND_PCTRL(ctls, k, "Sharpness map: (k * laplacian + gradient) / (k + 1)");
  BIND_PCTRL(ctls, p, "Sharpness estimator: sharpness map power");
  BIND_PCTRL(ctls, dscale, "Sharpness estimator: sharpness map downscale pyramid level");
  BIND_PCTRL(ctls, uscale, "Sharpness estimator: sharpness map upscale pyramid level");
  BIND_PCTRL(ctls, avgchannel, "Sharpness estimator: use grayscale sharpness map");
}

bool c_lpg_map_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, k);
    SERIALIZE_PROPERTY(settings, save, *this, p);
    SERIALIZE_PROPERTY(settings, save, *this, dscale);
    SERIALIZE_PROPERTY(settings, save, *this, uscale);
    SERIALIZE_PROPERTY(settings, save, *this, avgchannel);
    return true;
  }
  return false;
}

bool c_lpg_map_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  _m.create_map(image.getMat(), image);
  return true;
}

bool c_lpg_map_routine::copy_parameters_to_clipboard()
{
  if ( const auto & cb = get_ctrlbind_copy_to_clipboard_callback() ) {
    cb(_m.save_settings());
  }
  else {
    CF_ERROR("No clipboard callback available");
  }
  return false;
}

bool c_lpg_map_routine::paste_parameters_from_clipboard()
{
  if ( const auto & cb = get_ctrlbind_get_clipboard_text_callback() ) {
    return _m.load_settings(cb());
  }

  CF_ERROR("No clipboard callback available");
  return false;
}

