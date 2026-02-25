/*
 * c_rotate_image_routine.h
 *
 *  Created on: Jun 10, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rotate_image_routine_h__
#define __c_rotate_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_rotate_image_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_rotate_image_routine,
       "rotate", "call cv::rotate() on image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _rotation_angle = 0 ;
};

#endif /* __c_rotate_image_routine_h__ */
