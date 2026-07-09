/*
 * c_align_color_channels_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_align_color_channels_routine_h__
#define __c_align_color_channels_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/image_registration/c_align_color_channels.h>

class c_align_color_channels_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_align_color_channels_routine,
      "align_color_channels", "Align color channels to reference one");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_align_color_channels _algorithm;
  c_align_color_channels_options _opts;
  int reference_channel = 1;
};

#endif /* __c_align_color_channels_routine_h__ */
