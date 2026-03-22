/*
 * c_hdl_pixel_math_routine.h
 *
 *  Created on: Mar 22, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_pixel_math_routine_h__
#define __c_hdl_pixel_math_routine_h__

#include "c_hdl_processor_routine.h"
#include <core/proc/c_math_expression.h>

class c_hdl_pixel_math_routine :
    public c_hdl_processor_routine
{
public:
  DECLARE_HDL_PROCESSOR_CLASS_FACTORY(c_hdl_pixel_math_routine,
      "hdl_pixel_math",
      "Apply math formula for HDL pixel processing");

  void set_expression(const std::string & v)
  {
    _expression = v;
    _math.clear();
  }

  const std::string & expression() const
  {
    return _expression;
  }

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(c_hdl_data_frame * hdl) final;

protected:
  std::string _expression;
  cv::Mat3f _output_image;
  c_math_expression _math;
};

#endif /* __c_hdl_pixel_math_routine_h__ */
