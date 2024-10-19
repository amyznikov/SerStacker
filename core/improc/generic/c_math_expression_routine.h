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
    expression_ = v;
    expression_changed_ = true;
  }

  const std::string & expression() const
  {
    return expression_;
  }

  std::string helpstring();
  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  bool initialize() final;

protected:
  c_math_expression math_;
  std::string expression_;
  bool expression_changed_ = true;
};

#endif /* __c_math_expression_routine_h__ */
