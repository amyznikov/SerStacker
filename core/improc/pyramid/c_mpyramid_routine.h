/*
 * c_mpyramid_routine.h
 *
 *  Created on: Aug 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mpyramid_routine_h__
#define __c_mpyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_mpyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_mpyramid_routine,
      "mpyramid", "Display Laplacian Magnitude pyramid layers");

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

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  int minimum_image_size_ = 4;
  int display_pos_ = 0;
};

#endif /* __c_mpyramid_routine_h__ */
