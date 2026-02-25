/*
 * c_math_expression_routine.h
 *
 *  Created on: Oct 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_math_expression_routine_h__
#define __c_math_expression_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_math_expression.h>

class c_math_expression_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_math_expression_routine,
      "math", "c_math_expression_routine");

  void set_expression(const std::string & v)
  {
    _expression = v;
    _expression_changed = true;
  }

  const std::string & expression() const
  {
    return _expression;
  }

  std::string helpstring();
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  bool initialize() final;

protected:
  c_math_expression _math;
  std::string _expression;
  bool _expression_changed = true;
};

#endif /* __c_math_expression_routine_h__ */
