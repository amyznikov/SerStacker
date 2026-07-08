/*
 * c_adaptive_gaussian_blur_routine.h
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_adaptive_blend_filter_routine_h__
#define __c_adaptive_blend_filter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/adaptive_gaussian_blur.h>

class c_adaptive_gaussian_blur_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_adaptive_gaussian_blur_routine,
      "c_adaptive_gaussian_blur_routine", "Apply adaptive denoise filter");

  using DisplayType = ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_TYPE;

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double sigma_hpass = 0.75;
  double sigma_lpass = 2.0;
  double lpg_scale = 100;
  double lpgk = 0.5;
  DisplayType displayType = ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_FILTERED;
};

#endif /* __c_adaptive_blend_filter_routine_h__ */
