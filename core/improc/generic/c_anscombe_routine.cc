/*
 * c_anscombe_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_anscombe_routine.h"

void c_anscombe_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  c_anscombe_transform::getcontrols(ctls, ctx(&this_class::anscombe));
  ctlbind(ctls, "invert", ctx(&this_class::invert), "");
}

bool c_anscombe_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, anscombe, method);
    SERIALIZE_OPTION(settings, save, *this, invert);
    return true;
  }
  return false;
}

bool c_anscombe_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( invert ) {
    anscombe.inverse(image.getMatRef(), image.getMatRef());
  }
  else {
    anscombe.apply(image.getMatRef(), image.getMatRef());
  }

  return true;
}


