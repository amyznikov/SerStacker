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

  enum CHANNEL {
    IMAGE,
    MASK
  };

  void set_expression(const std::string & v)
  {
    expression_ = v;
    expression_changed_ = true;
  }

  const std::string & expression() const
  {
    return expression_;
  }

  void set_input_channel(CHANNEL v )
  {
    input_channel_ = v;
  }

  CHANNEL input_channel() const
  {
    return input_channel_;
  }

  void set_output_channel(CHANNEL v )
  {
    output_channel_ = v;
  }

  CHANNEL output_channel() const
  {
    return output_channel_;
  }

  std::string helpstring();
  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  bool initialize() override;

protected:
  c_math_expression math_;
  std::string expression_;
  bool expression_changed_ = true;
  CHANNEL input_channel_ = IMAGE;
  CHANNEL output_channel_ = IMAGE;
};

#endif /* __c_math_expression_routine_h__ */
