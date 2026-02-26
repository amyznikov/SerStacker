/*
 * c_lpg_map_routine.cc
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#include "c_lpg_map_routine.h"
#include <core/ctrlbind/ctrlbind.h>


void c_lpg_map_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  c_lpg_sharpness_measure::getcontrols(ctls, ctx(&this_class::_m));

  ctlbind_menu_button(ctls, "Options...", ctx);
    ctlbind_command_button(ctls, "Copy c_lpg_options to clipboard", ctx,
        std::function([](this_class * _ths) {
          ctlbind_copy_config_to_clipboard("c_lpg_options", _ths->_m.options());
          return false;
        }));
    ctlbind_command_button(ctls, "Paste c_lpg_options from clipboard", ctx,
        std::function([](this_class * _ths) {
          return ctlbind_paste_config_from_clipboard("c_lpg_options", &_ths->_m.options());
        }));

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
//
//bool c_lpg_map_routine::copy_parameters_to_clipboard()
//{
//  if ( const auto & cb = get_ctrlbind_copy_to_clipboard_callback() ) {
//    cb(_m.save_settings());
//  }
//  else {
//    CF_ERROR("No clipboard callback available");
//  }
//  return false;
//}
//
//bool c_lpg_map_routine::paste_parameters_from_clipboard()
//{
//  if ( const auto & cb = get_ctrlbind_get_clipboard_text_callback() ) {
//    return _m.load_settings(cb());
//  }
//
//  CF_ERROR("No clipboard callback available");
//  return false;
//}

