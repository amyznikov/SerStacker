/*
 * c_pixel_func_routine.h
 *
 *  Created on: May 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pixel_func_routine_h__
#define __c_pixel_func_routine_h__

#include <core/improc/c_image_processor.h>

class c_pixel_func_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_pixel_func_routine,
      "pixfunc", "Apply specified function to each image pixel<br>"
          "V' = c3 * func( (V - c1) * c2, [p0..pn] ) + c4");

  enum Function {
    Function_None,
    Function_sqrt,
    Function_sqr,
    Function_abs,
    Function_log,
    Function_exp,
    Function_inv,
    Function_sin,
    Function_cos,
    Function_asin,
    Function_acos,
    Function_asinh,
    Function_acosh,
    Function_pow,
    Function_poly,
  };

  void set_function(enum Function v)
  {
    function_ = v;
  }

  enum Function function() const
  {
    return function_;
  }

  void set_c1(double v)
  {
    c1_ = v;
  }

  double c1() const
  {
    return c1_;
  }


  void set_c2(double v)
  {
    c2_ = v;
  }

  double c2() const
  {
    return c2_;
  }

  void set_c3(double v)
  {
    c3_ = v;
  }

  double c3() const
  {
    return c3_;
  }

  void set_c4(double v)
  {
    c4_ = v;
  }

  double c4() const
  {
    return c4_;
  }

  void set_params(std::vector<double> & v)
  {
    params_ = v;
  }

  const std::vector<double> & params() const
  {
    return params_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  enum Function function_ = Function_None;
  double c1_ = 0;
  double c2_ = 1;
  double c3_ = 1;
  double c4_ = 0;

  std::vector<double> params_;

};

#endif /* __c_pixel_func_routine_h__ */
