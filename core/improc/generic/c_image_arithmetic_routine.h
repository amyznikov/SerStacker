/*
 * c_image_arithmetic_routine.h
 *
 *  Created on: Mar 22, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_arithmetic_routine_h__
#define __c_image_arithmetic_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_math_expression.h>

class c_image_arithmetic_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_image_arithmetic_routine,
      "image_arithmetic", "arithmetic between 2 images");

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
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  c_math_expression _math;
  std::string _expression;
  std::string _argname;
  std::string _outname;
};

#endif /* __c_image_arithmetic_routine_h__ */
