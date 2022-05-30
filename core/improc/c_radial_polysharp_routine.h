/*
 * c_radial_polysharp_routine.h
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_radial_polysharp_routine_h__
#define __c_radial_polysharp_routine_h__

#include "c_image_processor.h"

class c_radial_polysharp_routine :
    public c_image_processor_routine
{
public:
  typedef c_radial_polysharp_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("polysharp", "polysharp", "fft radial poly sharp",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_radial_polysharp_routine(bool enabled = true);

  void set_coeffs(std::vector<double> & v);
  const std::vector<double> & coeffs() const;
  std::vector<double> & coeffs();

  const std::vector<double> & profile_before() const;
  const std::vector<double> & profile_after() const;
  const std::vector<double> & profile_poly() const;

  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, coeffs, "");
  }

protected:
  std::vector<double> profile_before_;
  std::vector<double> profile_after_;
  std::vector<double> profile_poly_;
  std::vector<double> coeffs_;
};

#endif /* __c_radial_polysharp_routine_h__ */
