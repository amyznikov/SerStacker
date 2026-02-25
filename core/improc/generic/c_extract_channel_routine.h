/*
 * c_extract_channel_routine.h
 *
 *  Created on: Jan 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_extract_channel_routine_h__
#define __c_extract_channel_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>

class c_extract_channel_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_extract_channel_routine, "extract_channel",
      "Apply extract_channel() to extract single color channel from image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  enum color_channel_type _output_channel = color_channel_gray;
  enum PIXEL_DEPTH _output_depth = PIXEL_DEPTH_NO_CHANGE;
  double _output_scale = 1.;
  double _output_depth_scale = 1.0;

};

#endif /* __c_extract_channel_routine_h__ */
