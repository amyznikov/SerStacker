/*
 * c_test_scale_space_routine.h
 *
 *  Created on: Sep 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_test_scale_space_routine_h__
#define __c_test_scale_space_routine_h__

#include <core/improc/c_image_processor.h>

class c_test_scale_space_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_test_scale_space_routine,
       "test_scale_space", "");

  void set_minimum_image_size(int v);
  int minimum_image_size() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  void apply_functional();

protected:
  int minimum_image_size_ = 32;
};

#endif /* __c_test_scale_space_routine_h__ */
