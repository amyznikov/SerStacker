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

class c_fft_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_filter_routine,
      "fft_filter", "");

  enum FILTER {
    FILTER_GAUSSIAN,
    FILTER_LAPLACIAN,
    FILTER_RAMP,
    FILTER_BUTTERWORTH,
    FILTER_GAUSSIAN_SHARP,
    FILTER_LAPLACIAN_SHARP
  };

  enum DISPLAY {
    DISPLAY_SRC_IMAGE,
    DISPLAY_SRC_SPECTRUM_MODULE,
    DISPLAY_SRC_SPECTRUM_POWER,
    DISPLAY_FILTER_MODULE,
    DISPLAY_FILTER_POWER,
    DISPLAY_FILTERED_SPECTRUM_MODULE,
    DISPLAY_FILTERED_SPECTRUM_POWER,
    DISPLAY_FILTERED_IMAGE,

    DISPLAY_VLAP,
    //DISPLAY_PPS,

  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  FILTER _filterType = FILTER_GAUSSIAN;
  DISPLAY _display = DISPLAY_FILTERED_IMAGE;
  bool _ppsDecomposition = false;

  struct c_gaussian_filter_opts {
    double sigma = 1;
    double gain = 1;
  } gaussian;

  struct c_laplacian_filter_opts {
    double gain = 1;
  } laplacian;

  struct c_ramp_filter_opts {
    double gain = 1;
  } ramp;

  struct c_butterworth_filter_opts {
    double rc = 1;
    double gain = 1;
    int order = 4;
  } butterworth;

  struct c_gaussian_sharp_filter_opts {
    double sigma = 1;
    double gain = 1;
  } gaussian_sharp;

  struct c_laplacian_sharp_filter_opts {
    double gain = 50;
    double bwrc = 1;
    double bworder = 4;
  } laplacian_sharp;

  cv::Mat1f VLAP;
};

#endif /* __c_fft_gaussian_filter_routine_h__ */
