/*
 * c_vlo_pixel_selection_routine.h
 *
 *  Created on: Jan 3, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_pixel_selection_routine_h__
#define __c_vlo_pixel_selection_routine_h__

#include "c_vlo_processor_routine.h"
#include <core/proc/c_math_expression.h>

class c_vlo_pixel_selection_routine :
    public c_vlo_processor_routine
{
public:
  DECLARE_VLO_PROCESSOR_CLASS_FACTORY(c_vlo_pixel_selection_routine,
      "vlo_pixel_selection",
      "VLO pixels selection based on math expression");

  void set_expression(const std::string & v)
  {
    expression_ = v;
    expression_changed_ = true;
  }

  const std::string & expression() const
  {
    return expression_;
  }

  void set_mask_mode(c_vlo_frame::SELECTION_MASK_MODE v)
  {
    mask_mode_ = v;
  }

  c_vlo_frame::SELECTION_MASK_MODE mask_mode() const
  {
    return mask_mode_;
  }

  void set_invert_selection(bool v)
  {
    invert_selection_ = v;
  }

  bool invert_selection() const
  {
    return invert_selection_;
  }

  void get_parameters(std::vector<struct c_data_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_vlo_frame * vlo) override;
  std::string helpstring() const;

protected:
  std::string expression_;
  c_math_expression math_;
  cv::Mat3b selection_mask;
  c_vlo_frame::SELECTION_MASK_MODE mask_mode_ = c_vlo_frame::SELECTION_MASK_AND;
  int previous_vlo_scan_version_ = -1;
  bool invert_selection_ = false;
  bool expression_changed_ = true;
};

#endif /* __c_vlo_pixel_selection_routine_h__ */
