/*
 * c_set_pixels_routine.h
 *
 *  Created on: Mar 21, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_set_pixels_routine_h__
#define __c_set_pixels_routine_h__

#include <core/improc/c_image_processor.h>

class c_set_pixels_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_set_pixels_routine,
      "setTo",
      "image.setTo(value, mask)");

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  cv::Scalar _value;
  bool _invertMask = true;
};

#endif /* __c_set_pixels_routine_h__ */
