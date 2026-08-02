/*
 * c_histogram_white_balance_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_histogram_white_balance_routine.h"
#include <core/proc/histogram-tools.h>
//#include <core/proc/reduce_channels.h>

void c_histogram_white_balance_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "lclip [%%]", ctx(&this_class::_lclip), "lower quantile in percents");
   ctlbind(ctls, "hclip [%%]", ctx(&this_class::_hclip), "upper quantile in percents");
   ctlbind(ctls, "ignore mask", ctx(&this_class::_ignore_mask), "");
   ctlbind(ctls, "dump_parameters", ctx(&this_class::_dump_adjusted_parameters), "Dump adjusted parameters to debug log");
   ctlbind_command_button(ctls, "Copy scales to clipboard", ctx, &this_class::copy_scales_to_clipboard);
}

bool c_histogram_white_balance_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _lclip);
    SERIALIZE_OPTION(settings, save, *this, _hclip);
    SERIALIZE_OPTION(settings, save, *this, _ignore_mask);
    //SERIALIZE_OPTION(settings, save, *this, _dump_adjusted_parameters);

    return true;
  }
  return false;
}

bool c_histogram_white_balance_routine::copy_scales_to_clipboard()
{
  c_config cfg;
  c_config_setting root = cfg.root();

  save_settings(root, "scales", _computed_scales);
  save_settings(root, "shifts", _computed_shifts);
  if( !ctlbind_copy_config_to_clipboard(cfg) ) {
    CF_ERROR("c_histogram_white_balance_routine:: ctlbind_copy_config_to_clipboard() fails");
  }

  return false;
}

bool c_histogram_white_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.channels() < 2 ) {
    return true;
  }

  histogramClipWhiteBalance(image, _ignore_mask ? cv::noArray() : mask, image,
      0.01 * _lclip, 0.01 * _hclip,
      &_computed_scales, &_computed_shifts);

  if ( _dump_adjusted_parameters ) {
    CF_DEBUG("\nhistogram_white_balance: Scales={ %g %g %g %g } shifts={ %g %g %g %g }",
        _computed_scales[0], _computed_scales[1], _computed_scales[2], _computed_scales[3],
        _computed_shifts[0], _computed_shifts[1], _computed_shifts[2], _computed_shifts[3]);
  }

  return true;
}
