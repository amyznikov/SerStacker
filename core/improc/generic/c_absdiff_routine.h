/*
 * c_absdiff_routine.h
 *
 *  Created on: Jan 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_absdiff_routine_h__
#define __c_absdiff_routine_h__

#include <core/improc/c_image_processor.h>

class c_absdiff_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_absdiff_routine,
      "absdiff", "calls cv::absdiff(image, value)");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Scalar value;
};

#endif /* __c_absdiff_routine_h__ */
