/*
 * c_radial_polysharp_routine.cc
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#include "c_radial_polysharp_routine.h"

void c_radial_polysharp_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "coeffs", ctx(&this_class::_coeffs), "");
}

bool c_radial_polysharp_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _coeffs);
    return true;
  }
  return false;
}

bool c_radial_polysharp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  fftRadialPolySharp(image, image, _coeffs, _profile_before, _profile_after, _profile_poly);
  return true;
}

