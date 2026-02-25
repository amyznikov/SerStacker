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
    _coeffs = v;
  }

  const std::vector<double> & coeffs() const
  {
    return _coeffs;
  }

  std::vector<double> & coeffs()
  {
    return _coeffs;
  }

  const std::vector<double> & profile_before() const
  {
    return _profile_before;
  }

  const std::vector<double> & profile_after() const
  {
    return _profile_after;
  }

  const std::vector<double> & profile_poly() const
  {
    return _profile_poly;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::vector<double> _profile_before;
  std::vector<double> _profile_after;
  std::vector<double> _profile_poly;
  std::vector<double> _coeffs;
};

#endif /* __c_radial_polysharp_routine_h__ */
