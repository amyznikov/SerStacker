/*
 * c_color_transform_routine.h
 *
 *  Created on: Mar 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_color_transform_routine_h__
#define __c_color_transform_routine_h__

#include <core/improc/c_image_processor.h>

class c_color_transform_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_color_transform_routine,
      "color_transform", "Apply cv::transfrom() to image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);


protected:
  float _rb = 0;
  float _rg = 0;
  float _rr = 0;
};

#endif /* __c_color_transform_routine_h__ */
