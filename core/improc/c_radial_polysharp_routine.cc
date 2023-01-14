/*
 * c_radial_polysharp_routine.cc
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#include "c_radial_polysharp_routine.h"
#include <core/proc/fft.h>

c_radial_polysharp_routine::c_class_factory c_radial_polysharp_routine::class_factory;

c_radial_polysharp_routine::c_radial_polysharp_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_radial_polysharp_routine::ptr c_radial_polysharp_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_radial_polysharp_routine::set_coeffs(std::vector<double> & v)
{
  coeffs_ = v;
}

const std::vector<double> & c_radial_polysharp_routine::coeffs() const
{
  return coeffs_;
}

std::vector<double> & c_radial_polysharp_routine::coeffs()
{
  return coeffs_;
}

const std::vector<double> & c_radial_polysharp_routine::profile_before() const
{
  return profile_before_;
}

const std::vector<double> & c_radial_polysharp_routine::profile_after() const
{
  return profile_after_;
}

const std::vector<double> & c_radial_polysharp_routine::profile_poly() const
{
  return profile_poly_;
}


bool c_radial_polysharp_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, coeffs);

  return true;
}

bool c_radial_polysharp_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, *this, coeffs);

  return true;
}

bool c_radial_polysharp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  fftRadialPolySharp(image, image, coeffs_, profile_before_, profile_after_, profile_poly_);
  return true;
}
