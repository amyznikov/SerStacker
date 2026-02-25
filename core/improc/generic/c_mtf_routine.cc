/*
 * c_mtf_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_mtf_routine.h"

bool c_mtf_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _inputRange);
    SERIALIZE_OPTION(settings, save, *this, _outputRange);
    SERIALIZE_OPTION(settings, save, *this, _shadows);
    SERIALIZE_OPTION(settings, save, *this, _highlights);
    SERIALIZE_OPTION(settings, save, *this, _midtones);
    return true;
  }
  return false;
}

void c_mtf_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "clip input range", ctx(&this_class::_inputRange), "");
  ctlbind(ctls, "stretch output range", ctx(&this_class::_outputRange), "");
  ctlbind_slider_spinbox(ctls, "shadows", ctx(&this_class::_shadows), 0.0, 1.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "highlights", ctx(&this_class::_highlights), 0.0, 1.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "midtones", ctx(&this_class::_midtones), 0.0, 1.0, 0.001, "");
}


bool c_mtf_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  _mtf.set_input_range(_inputRange[0], _inputRange[1]);
  _mtf.set_output_range(_outputRange[0], _outputRange[1]);
  _mtf.set_shadows(_shadows);
  _mtf.set_highlights(_highlights);
  _mtf.set_midtones(_midtones);

  return _mtf.apply(image.getMat(), image);
}

