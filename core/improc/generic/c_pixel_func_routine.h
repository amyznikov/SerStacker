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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  enum Function _function = Function_None;
  double _c1 = 0;
  double _c2 = 1;
  double _c3 = 1;
  double _c4 = 0;
  std::vector<double> _params;
};

#endif /* __c_pixel_func_routine_h__ */
