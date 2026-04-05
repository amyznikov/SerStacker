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
      "");


  enum InputType
  {
    InputType_Image = DisplayType_Image,
    InputType_PointCloud = DisplayType_PointCloud,
  };

  InputType input_type() const
  {
    return _input_type;
  }

  void set_input_type(InputType v)
  {
    _input_type = v;
    _expression_changed = true;
  }

  const std::string & input() const
  {
    return _input;
  }

  void set_input(const std::string & v)
  {
    _input = v;
  }

  const std::string & output() const
  {
    return _output;
  }

  void set_output(const std::string & v)
  {
    _output = v;
  }

  void set_output_depth(enum PIXEL_DEPTH v)
  {
    _output_depth = v;
  }

  enum PIXEL_DEPTH output_depth() const
  {
    return _output_depth;
  }

  void set_expression(const std::string & v)
  {
    _expression = v;
    _expression_changed = true;
  }

  const std::string & expression() const
  {
    return _expression;
  }


  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool process(c_data_frame::sptr & dataframe) final;
  bool serialize(c_config_setting settings, bool save) final;

protected:
  bool initialize() final;

protected:
  std::string _input;
  std::string _output;
  std::string _helpstring;
  InputType _input_type = InputType_Image;
  PIXEL_DEPTH _output_depth = PIXEL_DEPTH_NO_CHANGE;

  std::string _expression;
  mutable c_math_expression _math;
  bool _expression_changed = true;

};

#endif /* __c_pixel_processor_routine_h__ */
