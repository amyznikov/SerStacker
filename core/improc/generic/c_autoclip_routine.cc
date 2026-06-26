/*
 * c_autoclip_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_autoclip_routine.h"
#include <core/proc/histogram-tools.h>

void c_autoclip_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "clipRange [%]", ctx(&this_class::_clipRange), "Clip range in percents [qLow;qHigh]");
  ctlbind(ctls, "outputRange", ctx(&this_class::_outputRange), "Output range");
  ctlbind(ctls, "autoOutputRange", ctx(&this_class::_autoOutputRange), "Auto output range depending on ddepth");
  ctlbind(ctls, "ddepth", ctx(&this_class::_ddepth), "Target pixel depth");
}

bool c_autoclip_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _clipRange);
    SERIALIZE_OPTION(settings, save, *this, _outputRange);
    SERIALIZE_OPTION(settings, save, *this, _autoOutputRange);
    SERIALIZE_OPTION(settings, save, *this, _ddepth);
    return true;
  }
  return false;
}

bool c_autoclip_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const double qlow = 0.01 * _clipRange[0];
  const double qhigh = 0.01 * _clipRange[1];
  const int ddepth = _ddepth < 0 ? image.depth() : _ddepth;

  double omin = _outputRange[0];
  double omax = _outputRange[1];
  if ( _autoOutputRange ) {
    getDataRangeForPixelDepth(ddepth,&omin, &omax);
  }

  return autoClip(image.getMat(), mask, image,
      qlow, qhigh, omin, omax,
      nullptr, nullptr,
      ddepth);
}

