/*
 * c_scale_channels_routine.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_scale_channels_routine_h__
#define __c_scale_channels_routine_h__

#include <core/improc/c_image_processor.h>

class c_scale_channels_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_scale_channels_routine,
      "scale_channels", "stretch color channels: v' = stretch * v + shift");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Scalar _stretch = cv::Scalar::all(1);
  cv::Scalar _shift = cv::Scalar::all(0);
};

#endif /* __c_scale_channels_routine_h__ */
