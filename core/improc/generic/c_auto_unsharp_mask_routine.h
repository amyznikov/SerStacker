/*
 * c_auto_unsharp_mask_routine.h
 *
 *  Created on: Sep 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_auto_unsharp_mask_routine_h__
#define __c_auto_unsharp_mask_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/focus.h>

class c_auto_unsharp_mask_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_auto_unsharp_mask_routine,
      "auto_unsharp_mask", "Apply auto unsharp mask to image");

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

  void set_sigma(double v)
  {
    _measure.set_sigma(v);
  }

  double sigma() const
  {
    return _measure.sigma();
  }

  void set_norm_type(cv::NormTypes v)
  {
    _measure.set_norm_type(v);
  }

  cv::NormTypes norm_type() const
  {
    return _measure.norm_type();
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_sharpness_norm_measure _measure;
  double _target_sharpness = 0.5;
  double _alpha_factor = 1.5;
  double _outmin = -1, _outmax = -1;
  double _blur_color_channels = 0;
  COLOR_CHANNEL _channel = COLOR_CHANNEL_ALL;
};


#endif /* __c_auto_unsharp_mask_routine_h__ */
