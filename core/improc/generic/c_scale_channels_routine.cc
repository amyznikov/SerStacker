/*
 * c_scale_channels_routine.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "c_scale_channels_routine.h"
#include <core/proc/histogram-tools.h>

void c_scale_channels_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "stretch (B;G;R;A)", ctx(&this_class::_stretch), "Scales applied to color channels");
  ctlbind(ctls, "shifts  (B;G;R;A)", ctx(&this_class::_shift), "Offsets added to color channels");

  ctlbind(ctls, "Auto white balance", ctx(&this_class::_auto_white_balance), "Set checked for auto white balance");
  ctlbind(ctls, "auto clip (l;h) [%]:", ctx(&this_class::_auto_white_balance_clips),
      "Low and High quantiles for auto white balance, in percents");

  ctlbind_command_button(ctls, "Paste scales from clipboard", ctx, &this_class::paste_scales_from_clipboard);
}

bool c_scale_channels_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _stretch);
    SERIALIZE_OPTION(settings, save, *this, _shift);
    SERIALIZE_OPTION(settings, save, *this, _auto_white_balance_clips);
    return true;
  }
  return false;
}

bool c_scale_channels_routine::paste_scales_from_clipboard()
{
  c_config cfg;
  if( !ctlbind_paste_config_to_clipboard(cfg) ) {
    CF_ERROR("c_histogram_white_balance_routine:: ctlbind_paste_config_to_clipboard() fails");
  }
  else {
    c_config_setting root = cfg.root();
    load_settings(root, "scales", &_stretch);
    load_settings(root, "shifts", &_shift);
    return true;
  }
  return false;
}

bool c_scale_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _auto_white_balance && image.channels() > 1 ) {
    const double qlow = 0.01 * _auto_white_balance_clips[0];
    const double qhigh = 0.01 * _auto_white_balance_clips[1];
    histogramClipWhiteBalance(image, mask, qlow, qhigh, _stretch, _shift);
    set_has_contol_changes(true);
  }

  return applyChannelTransform(image, image, _stretch, _shift);
}
