/*
 * c_transpose_image_routine.h
 *
 *  Created on: Nov 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_transpose_image_routine_h__
#define __c_transpose_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_transpose_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_transpose_image_routine,
       "transpose", "Call <strong>cv::transpose()</strong> on image");

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

};

#endif /* __c_transpose_image_routine_h__ */
