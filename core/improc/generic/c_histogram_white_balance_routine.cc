/*
 * c_histogram_white_balance_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_histogram_white_balance_routine.h"
#include <core/proc/white_balance/histogram_white_balance.h>
#include <core/proc/reduce_channels.h>

void c_histogram_white_balance_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "lclip", ctx(&this_class::_lclip), "");
   ctlbind(ctls, "hclip", ctx(&this_class::_hclip), "");
   ctlbind(ctls, "enable_threshold", ctx(&this_class::_enable_threshold), "");
   ctlbind(ctls, "threshold", ctx(&this_class::_threshold), "");
}

bool c_histogram_white_balance_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _lclip);
    SERIALIZE_OPTION(settings, save, *this, _hclip);
    SERIALIZE_OPTION(settings, save, *this, _enable_threshold);
    SERIALIZE_OPTION(settings, save, *this, _threshold);
    return true;
  }
  return false;
}

bool c_histogram_white_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.channels() < 2 ) {
    return true;
  }

  cv::Mat objmask;

  if ( !_enable_threshold ) {
    objmask = mask.getMat();
  }
  else {
    cv::compare(image, _threshold, objmask, cv::CMP_GE);

    if ( objmask.channels() > 1 ) {
      reduce_color_channels(objmask, cv::REDUCE_MIN);
    }

    if ( !mask.empty() ) {
      cv::bitwise_and(mask, objmask, objmask);
    }
  }

  return histogram_white_balance(image.getMatRef(),
      objmask,
      image.getMatRef(),
      _lclip,
      _hclip);
}
