/*
 * c_local_variance_map_routine.cc
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#include "c_local_variance_map_routine.h"

bool c_local_variance_map_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    serialize_local_variance_map_options(settings, save, _opts);
    return true;
  }
  return false;
}

void c_local_variance_map_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, ctx(&this_class::_opts));
  ctlbind_menu_button(ctls, "Options...", ctx);
  ctlbind_item(ctls, "Copy c_local_variance_map to clipboard", ctx, [](this_class * _ths) {
    return ctlbind_copy_config_to_clipboard("c_local_variance_map", _ths->_opts), false;
  });
  ctlbind_item(ctls, "Paste c_local_variance_map from clipboard", ctx, [](this_class * _ths) {
    return ctlbind_paste_config_from_clipboard("c_local_variance_map", &_ths->_opts);
  });
}

bool c_local_variance_map_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  c_local_variance_sharpness_measure m(_opts);
  return m.create_map(image, image);
}



