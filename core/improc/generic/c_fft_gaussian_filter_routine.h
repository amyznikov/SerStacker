/*
 * c_fft_gaussian_filter_routine.h
 *
 *  Created on: Jun 11, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_gaussian_filter_routine_h__
#define __c_fft_gaussian_filter_routine_h__

#include <core/improc/c_image_processor.h>

class c_fft_gaussian_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_gaussian_filter_routine,
      "fft_gaussian_filter", "");

  enum DISPLAY {
    DISPLAY_SRC_IMAGE,
    DISPLAY_SRC_SPECTRUM_MODULE,
    DISPLAY_SRC_SPECTRUM_POWER,
    DISPLAY_GAUSSIAN_MODULE,
    DISPLAY_GAUSSIAN_POWER,
    DISPLAY_FILTERED_SPECTRUM_MODULE,
    DISPLAY_FILTERED_SPECTRUM_POWER,
    DISPLAY_FILTERED_IMAGE
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_FILTERED_IMAGE;
  double _gsigma = 1;
  double _gain = 1;
  double _gain_cutoff = 0.7;
};

#endif /* __c_fft_gaussian_filter_routine_h__ */
