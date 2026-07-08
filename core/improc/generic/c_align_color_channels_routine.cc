/*
 * c_align_color_channels_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_align_color_channels_routine.h"
#include <core/proc/reduce_channels.h>

void c_align_color_channels_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "reference channel", CTL_CONTEXT(ctx,reference_channel), "");
  base::getcontrols(ctls, ctx);
  ctlbind_expandable_group(ctls, "Algorithm", "Algorithm parameters");
    ctlbind(ctls, CTL_CONTEXT(ctx, _opts));
  ctlbind_end_group(ctls);
}

bool c_align_color_channels_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, reference_channel);
    return serialize_align_color_channels_options(settings, save, _opts);
  }
  return false;
}

bool c_align_color_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return _algorithm.align(image, image, _opts, reference_channel,
      _ignore_mask ? cv::noArray() : mask,
      _ignore_mask ? cv::noArray() : mask);
}

