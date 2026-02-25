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
       "transpose", "call cv::transpose() on image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

};

#endif /* __c_transpose_image_routine_h__ */
