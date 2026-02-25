/*
 * c_set_luminance_channel_routine.h
 *
 *  Created on: May 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_set_luminance_channel_routine_h__
#define __c_set_luminance_channel_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/extract_channel.h>

class c_set_luminance_channel_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_set_luminance_channel_routine,
      "set_luminance_channel",
      "Set specified color channel as luminance.<br>"
      "For non-linear color spaces like Lab/Luv the input image must be normalized to standard range "
      "(e.g. [0..1] for CV_32F)");

  enum Colorspace {
    Colorspace_Lab,
    Colorspace_Luv,
    Colorspace_HSV,
    Colorspace_HLS,
    Colorspace_YCrCb,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  enum color_channel_type _luminance_channel = color_channel_red;
  enum Colorspace _colorspace = Colorspace_Lab;
  double _usharp_sigma = 1;
  double _usharp_alpha = 0;
  double _usharp_outmin = -1;
  double _usharp_outmax = -1;
};

#endif /* __c_set_luminance_channel_routine_h__ */
