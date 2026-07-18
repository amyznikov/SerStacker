/*
 * c_histogram_white_balance_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_histogram_white_balance_routine.h"
#include <core/proc/histogram-tools.h>
#include <core/proc/reduce_channels.h>

void c_histogram_white_balance_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "lclip [%%]", ctx(&this_class::_lclip), "lower quantile in percents");
   ctlbind(ctls, "hclip [%%]", ctx(&this_class::_hclip), "upper quantile in percents");
   ctlbind(ctls, "ignore mask", ctx(&this_class::_ignore_mask), "");
   ctlbind(ctls, "dump_parameters", ctx(&this_class::_dump_adjusted_parameters), "Dump adjusted parameters to debug log");
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

bool c_histogram_white_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.channels() < 2 ) {
    return true;
  }

  cv::Scalar scales, shifts;

  histogramClipWhiteBalance(image, _ignore_mask ? cv::noArray() : mask, image,
      0.01 * _lclip, 0.01 * _hclip,
      &scales, &shifts);

  if ( _dump_adjusted_parameters ) {
    CF_DEBUG("\nhistogram_white_balance: Scales={ %g %g %g %g } shifts={ %g %g %g %g }",
        scales[0], scales[1], scales[2], scales[3],
        shifts[0], shifts[1], shifts[2], shifts[3]);
  }

  return true;
}
