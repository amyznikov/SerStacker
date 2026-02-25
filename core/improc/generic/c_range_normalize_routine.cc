/*
 * c_range_normalize_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_range_normalize_routine.h"

void c_range_normalize_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "input_min", ctx(&this_class::_input_min), "");
  ctlbind(ctls, "input_max", ctx(&this_class::_input_max), "");
  ctlbind(ctls, "output_min", ctx(&this_class::_output_min), "");
  ctlbind(ctls, "output_max", ctx(&this_class::_output_max), "");
  ctlbind(ctls, "auto_input_range", ctx(&this_class::_auto_input_range), "");
}

bool c_range_normalize_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _auto_input_range);
    SERIALIZE_OPTION(settings, save, *this, _input_min);
    SERIALIZE_OPTION(settings, save, *this, _input_max);
    SERIALIZE_OPTION(settings, save, *this, _output_min);
    SERIALIZE_OPTION(settings, save, *this, _output_max);
    return true;
  }
  return false;
}

bool c_range_normalize_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  bool fOk;

  // CF_DEBUG("auto_input_range_=%d", auto_input_range_);

  if( _auto_input_range ) {

    // CF_DEBUG("normalize_minmax(output_min_=%g, output_max_=%g)", output_min_, output_max_);
    fOk = normalize_minmax(image.getMatRef(),
        image.getMatRef(),
        _output_min,
        _output_max,
        mask);
  }
  else {

    // CF_DEBUG("normalize_image(input_min_=%g input_max_=%g output_min_=%g, output_max_=%g)", input_min_, input_max_, output_min_, output_max_);
    fOk = normalize_image(image.getMatRef(),
        _input_min,
        _input_max,
        _output_min,
        _output_max,
        mask.getMatRef());

  }

  return fOk;
}
