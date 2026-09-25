/*
 * c_fft_autosharp_routine.h
 *
 *  Created on: Jun 5, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_autosharp_routine_h__
#define __c_fft_autosharp_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_fft_autosharp.h>

class c_fft_autosharp_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_autosharp_routine,
      "fft_autosharp", "Auto sharpen raw stack with FFT");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  bool initialize() final;
  void state_changed() final;

protected:
  c_fft_autosharp_options opts;
  c_fft_autosharp_debug_options debug_opts;
  c_fft_autosharp autosharp;
  FFT_AUTOSHARP_OUTPUT_DISPLAY _display = FFT_AUTOSHARP_DISPLAY_RESTORED_IMAGE;
  bool _centerDC = true;
};

#endif /* __c_fft_autosharp_routine_h__ */
