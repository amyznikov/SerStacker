/*
 * c_melp_pyramid_routine.h
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_melp_pyramid_routine_h__
#define __c_melp_pyramid_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/laplacian_pyramid.h>

class c_melp_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_melp_pyramid_routine,
      "melp", "Display MeLP Pyramid layers");

  void set_min_image_size(int v)
  {
    minimum_image_size_ = v;
  }

  int min_image_size() const
  {
    return minimum_image_size_;
  }

  void set_display_row(int v)
  {
    display_row_ = v;
  }

  int display_row() const
  {
    return display_row_;
  }

  void set_display_col(int v)
  {
    display_col_ = v;
  }

  int display_col() const
  {
    return display_col_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  int minimum_image_size_ = 4;
  int display_row_ = 0;
  int display_col_ = 0;
  c_melp_pyramid pyramid_;
};

#endif /* __c_melp_pyramid_routine_h__ */
