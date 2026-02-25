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
    _expression = v;
    _expression_changed = true;
  }

  const std::string & expression() const
  {
    return _expression;
  }

  void set_mask_mode(c_data_frame::SELECTION_MASK_MODE v)
  {
    _mask_mode = v;
  }

  c_data_frame::SELECTION_MASK_MODE mask_mode() const
  {
    return _mask_mode;
  }

  void set_invert_selection(bool v)
  {
    _invert_selection = v;
  }

  bool invert_selection() const
  {
    return _invert_selection;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(c_video_frame * vlo) final;
  std::string helpstring();

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
  {
    ctlbind(ctls, "expression", ctx, &this_class::expression, &this_class::set_expression, "", &this_class::helpstring);
    ctlbind(ctls, "invert_selection", ctx, &this_class::invert_selection, &this_class::set_invert_selection);
    ctlbind(ctls, "mask_mode", ctx, &this_class::mask_mode, &this_class::set_mask_mode, "selection combine mode");
  }

protected:
  std::string _expression;
  c_math_expression _math;
  cv::Mat _current_image;
  cv::Mat1b _current_mask;
  c_data_frame::SELECTION_MASK_MODE _mask_mode = c_data_frame::SELECTION_MASK_AND;
  int _previous_vlo_scan_version = -1;
  bool _invert_selection = false;
  bool _expression_changed = true;
  bool _initialized = false;
};

#endif /* __c_image_pixels_selection_routine_h__ */
