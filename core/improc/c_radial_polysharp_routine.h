/*
 * c_radial_polysharp_routine.h
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_radial_polysharp_routine_h__
#define __c_radial_polysharp_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/fft.h>

class c_radial_polysharp_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_radial_polysharp_routine, "polysharp",
      "fft radial poly sharp");

  void set_coeffs(std::vector<double> & v)
  {
    coeffs_ = v;
  }

  const std::vector<double> & coeffs() const
  {
    return coeffs_;
  }

  std::vector<double> & coeffs()
  {
    return coeffs_;
  }

  const std::vector<double> & profile_before() const
  {
    return profile_before_;
  }

  const std::vector<double> & profile_after() const
  {
    return profile_after_;
  }

  const std::vector<double> & profile_poly() const
  {
    return profile_poly_;
  }


  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, coeffs, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, coeffs);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    fftRadialPolySharp(image, image, coeffs_, profile_before_, profile_after_, profile_poly_);
    return true;
  }


protected:
  std::vector<double> profile_before_;
  std::vector<double> profile_after_;
  std::vector<double> profile_poly_;
  std::vector<double> coeffs_;
};

#endif /* __c_radial_polysharp_routine_h__ */
