/*
 * c_image_pixels_selection_routine.h
 *
 *  Created on: Jan 7, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_pixels_selection_routine_h__
#define __c_image_pixels_selection_routine_h__

#include "c_video_frame_processor_routine.h"
#include <core/proc/c_math_expression.h>

class c_image_pixels_selection_routine :
    public c_video_frame_processor_routine
{
public:
  DECLARE_VIDEO_FRAME_PROCESSOR_CLASS_FACTORY(c_image_pixels_selection_routine,
      "pixels_selection",
      "Image pixels selection based on math expression");

  void set_expression(const std::string & v)
  {
    expression_ = v;
    expression_changed_ = true;
  }

  const std::string & expression() const
  {
    return expression_;
  }

  void set_mask_mode(c_data_frame::SELECTION_MASK_MODE v)
  {
    mask_mode_ = v;
  }

  c_data_frame::SELECTION_MASK_MODE mask_mode() const
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

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_video_frame * vlo) override;
  std::string helpstring() const;

protected:
  std::string expression_;
  c_math_expression math_;
  cv::Mat current_image_;
  cv::Mat1b current_mask_;
  c_data_frame::SELECTION_MASK_MODE mask_mode_ = c_data_frame::SELECTION_MASK_AND;
  int previous_vlo_scan_version_ = -1;
  bool invert_selection_ = false;
  bool expression_changed_ = true;
  bool initialized_ = false;
};

#endif /* __c_image_pixels_selection_routine_h__ */
