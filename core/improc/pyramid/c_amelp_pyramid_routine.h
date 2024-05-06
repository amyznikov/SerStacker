/*
 * c_amelp_pyramid_routine.h
 *
 *  Created on: May 6, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_amelp_pyramid_routine_h__
#define __c_amelp_pyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_amelp_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_amelp_pyramid_routine,
      "amelp_pyramid", "Display amelp pyramid layers");

  void set_scale_factor(double v)
  {
    scale_factor_ = v;
  }

  double scale_factor() const
  {
    return scale_factor_;
  }

  void set_max_level(int v)
  {
    max_level_ = v;
  }

  int max_level() const
  {
    return max_level_;
  }

  void set_display_pos(int v)
  {
    display_pos_ = v;
  }

  int display_pos() const
  {
    return display_pos_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  std::vector<cv::Mat> pyramid_;
  double scale_factor_ = 0.75;
  int display_pos_ = 0;
  int max_level_ = 3;
};

#endif /* __c_amelp_pyramid_routine_h__ */
