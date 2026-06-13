/*
 * c_fft_unsharp_filter_routine.h
 *
 *  Created on: Jun 13, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_unsharp_filter_routine_h__
#define __c_fft_unsharp_filter_routine_h__

#include <core/improc/c_image_processor.h>

class c_fft_unsharp_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_unsharp_filter_routine,
      "fft_unsharp_filter", "");

  enum FILTER {
    FILTER_GAUSSIAN,
    FILTER_BUTTERWORTH,
  };

  enum DISPLAY {
    DISPLAY_SRC_IMAGE,
    DISPLAY_SRC_SPECTRUM_MODULE,
    DISPLAY_SRC_SPECTRUM_POWER,
    DISPLAY_FILTER_MODULE,
    DISPLAY_FILTER_POWER,
    DISPLAY_FILTERED_SPECTRUM_MODULE,
    DISPLAY_FILTERED_SPECTRUM_POWER,
    DISPLAY_FILTERED_IMAGE
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  FILTER _filter = FILTER_GAUSSIAN;
  DISPLAY _display = DISPLAY_FILTERED_IMAGE;

  struct c_gaussian_filter_opts {
    double sigma = 1;
    double gain = 10;
  } gaussian_filter;

  struct c_butterworth_filter_opts {
    double rc = 1;
    double order = 2;
    double gain = 10;
  } butterworth_filter;


  double _gain_cutoff = 0.7;
};

#endif /* __c_fft_unsharp_filter_routine_h__ */
