/*
 * c_pixel_processor_routine.h
 *
 *  Created on: Jan 26, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pixel_processor_routine_h__
#define __c_pixel_processor_routine_h__

#include "c_data_frame_processor.h"
#include <core/proc/c_math_expression.h>
#include <core/proc/pixtype.h>

class c_pixel_processor_routine :
    public c_data_frame_processor_routine
{
public:
  DECLARE_DATA_PROCESSOR_CLASS_FACTORY(c_pixel_processor_routine,
      "pixel_processor",
      "c_pixel_processor_routine");


  enum InputType
  {
    InputType_Image = DisplayType_Image,
    InputType_PointCloud = DisplayType_PointCloud,
  };

  InputType input_type() const
  {
    return input_type_;
  }

  void set_input_type(InputType v)
  {
    input_type_ = v;
    expression_changed_ = true;
  }

  const std::string & input() const
  {
    return input_;
  }

  void set_input(const std::string & v)
  {
    input_ = v;
  }

  const std::string & output() const
  {
    return output_;
  }

  void set_output(const std::string & v)
  {
    output_ = v;
  }

  void set_output_depth(enum PIXEL_DEPTH v)
  {
    output_depth_ = v;
  }

  enum PIXEL_DEPTH output_depth() const
  {
    return output_depth_;
  }

  void set_expression(const std::string & v)
  {
    expression_ = v;
    expression_changed_ = true;
  }

  const std::string & expression() const
  {
    return expression_;
  }


  bool process(c_data_frame::sptr & dataframe) override;
  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  std::string helpstring() const;

protected:
  std::string input_;
  std::string output_;
  InputType input_type_ = InputType_Image;
  PIXEL_DEPTH output_depth_ = PIXEL_DEPTH_NO_CHANGE;

  std::string expression_;
  mutable c_math_expression math_;
  bool expression_changed_ = true;
  bool initialized_ = false;

};

#endif /* __c_pixel_processor_routine_h__ */
