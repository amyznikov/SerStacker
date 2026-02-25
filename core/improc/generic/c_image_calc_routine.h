/*
 * c_image_calc_routine.h
 *
 *  Created on: May 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_calc_routine_h__
#define __c_image_calc_routine_h__

#include <core/improc/c_image_processor.h>

class c_image_calc_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_image_calc_routine,
      "c_image_calc", "Arithmetic operation for current image Ic with argument image Ia<br>"
          "Ic' = func(Ic, Ia)");

  enum Function {
    Function_None,
    Function_add,
    Function_subtract,
    Function_absdiff,
    Function_multiply,
    Function_divide,
    Function_max,
    Function_min,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  enum Function _function = Function_None;
  std::string _argname;

};

#endif /* __c_image_calc_routine_h__ */
