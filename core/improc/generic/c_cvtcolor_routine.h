/*
 * c_cvtcolor_routine.h
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cvtcolor_routine_h__
#define __c_cvtcolor_routine_h__

#include <core/improc/c_image_processor.h>

class c_cvtcolor_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_cvtcolor_routine,
      "cvtcolor", "Apply cv::cvtColor() to image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  enum cv::ColorConversionCodes _conversion = cv::COLOR_COLORCVT_MAX;
};


#endif /* __c_cvtcolor_routine_h__ */
