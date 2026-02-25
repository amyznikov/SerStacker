/*
 * c_autoclip_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_autoclip_routine.h"

void c_autoclip_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "lclip", ctx(&this_class::_lclip), "");
  ctlbind(ctls, "hclip", ctx(&this_class::_hclip), "");
}

bool c_autoclip_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _lclip);
    SERIALIZE_OPTION(settings, save, *this, _hclip);
    return true;
  }
  return false;
}

bool c_autoclip_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  double omin = 0, omax = 1;

  switch (image.depth()) {
    case CV_8U:
      omin = 0, omax = UINT8_MAX;
      break;
    case CV_8S:
      omin = INT8_MIN, omax = INT8_MAX;
      break;
    case CV_16U:
      omin = 0, omax = UINT16_MAX;
      break;
    case CV_16S:
      omin = INT16_MIN, omax = INT16_MAX;
      break;
    case CV_32S:
      omin = INT32_MIN, omax = INT32_MAX;
      break;
      break;
  }

  return autoclip(image.getMatRef(),
      _ignore_mask ? cv::noArray() : mask,
      _lclip, _hclip,
      omin, omax);
}

