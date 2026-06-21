/*
 * c_anscombe_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_anscombe_routine.h"
#include <core/debug.h>

void c_anscombe_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "inverse", ctx(&this_class::inverse), "");
  c_anscombe_transform::getcontrols(ctls, ctx(&this_class::anscombe));
}

bool c_anscombe_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, anscombe, method);
    SERIALIZE_OPTION(settings, save, *this, inverse);
    if ( auto group = SERIALIZE_GROUP(settings, save, "opts") ) {
      c_anscombe_transform_options & opts = anscombe.opts();
      SERIALIZE_OPTION(settings, save, opts.generalized, g);
      SERIALIZE_OPTION(settings, save, opts.generalized, c);
      SERIALIZE_OPTION(settings, save, opts.generalized, auto_estimate);
      SERIALIZE_OPTION(settings, save, opts.generalized, dump_estimated_params);
    }

    return true;
  }
  return false;
}

bool c_anscombe_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !inverse ) {
    anscombe.apply(image, image);
  }
  else {
    anscombe.inverse(image, image);
  }

  return true;
}


