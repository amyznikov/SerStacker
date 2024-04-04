/*
 * c_vlo_pixel_math_routine.h
 *
 *  Created on: Feb 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_pixel_math_routine_h__
#define __c_vlo_pixel_math_routine_h__

#include "c_vlo_processor_routine.h"
#include <core/proc/c_math_expression.h>

class c_vlo_pixel_math_routine :
    public c_vlo_processor_routine
{
public:
  DECLARE_VLO_PROCESSOR_CLASS_FACTORY(c_vlo_pixel_math_routine,
      "vlo_pixel_processor",
      "VLO pixels processor using math expression");

  void set_expression(const std::string & v)
  {
    expression_ = v;
    expression_changed_ = true;
  }

  const std::string & expression() const
  {
    return expression_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_vlo_data_frame * vlo) override;
  std::string helpstring() const;

protected:
  std::string expression_;
  cv::Mat3f output_image_;
  mutable c_math_expression math_;
  bool expression_changed_ = true;

};

#endif /* __c_vlo_pixel_math_routine_h__ */
