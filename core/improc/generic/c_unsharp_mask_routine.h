/*
 * c_unsharp_mask_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_unsharp_mask_routine_h__
#define __c_unsharp_mask_routine_h__

#include <core/improc/c_image_processor.h>

class c_unsharp_mask_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_unsharp_mask_routine,
      "unsharp_mask", "Apply unsharp mask to image");

  enum COLOR_CHANNEL {
    COLOR_CHANNEL_ALL = -1,
    COLOR_CHANNEL_0 = 0,
    COLOR_CHANNEL_1 = 1,
    COLOR_CHANNEL_2 = 2,
    COLOR_CHANNEL_3 = 3,
    COLOR_CHANNEL_YCrCb = 4,
    COLOR_CHANNEL_Lab = 5,
    COLOR_CHANNEL_Luv = 6,
    COLOR_CHANNEL_HSV = 7,
    COLOR_CHANNEL_HLS = 8,
  };


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _sigma = 1;
  double _alpha = 0.9;
  double _outmin = -1;
  double _outmax = -1;
  COLOR_CHANNEL _channel = COLOR_CHANNEL_ALL;
  double _blur_color_channels = 0;
};

#endif /* __c_unsharp_mask_routine_h__ */
