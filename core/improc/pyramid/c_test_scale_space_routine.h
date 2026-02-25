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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  void apply_functional();

protected:
  int _minimum_image_size = 32;
};

#endif /* __c_test_scale_space_routine_h__ */
