/*
 * c_morph_gradient_pyramid_routine.h
 *
 *  Created on: Sep 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_morph_gradient_pyramid_routine_h__
#define __c_morph_gradient_pyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_morph_gradient_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_morph_gradient_pyramid_routine,
      "morph_gradient_pyramid", "Display morphological gradient Pyramid layers");

  void set_min_image_size(int v)
  {
    minimum_image_size_ = v;
  }

  int min_image_size() const
  {
    return minimum_image_size_;
  }

  void set_display_pos(int v)
  {
    display_pos_ = v;
  }

  int display_pos() const
  {
    return display_pos_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  std::vector<cv::Mat> pyramid_;
  int display_pos_ = 0;
  int minimum_image_size_ = 16;
};

#endif /* __c_morph_gradient_pyramid_routine_h__ */
