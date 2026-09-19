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
#include <core/proc/extract_channel.h>
#include <core/proc/c_anscombe_transform.h>
#include <core/proc/pixtype.h>

class c_fft_autosharp_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_autosharp_routine,
      "fft_autosharp", "Auto sharpen raw stack with FFT");

  enum DISPLAY {
    DISPLAY_SRC_IMAGE = 0,
    DISPLAY_RESTORED_IMAGE,
    DISPLAY_P_SPECTRUM,
    DISPLAY_S_SPECTRUM,
    DISPLAY_V_SPECTRUM,
    DISPLAY_FILTER,
    DISPLAY_RESTORED_SPECTRUM,
  };

  enum INPAINT_METHOD {
    INPAINT_DISABLED = 0,
    LINEAR_INTERPOLATION_INPAINT,
    AVERAGE_PYRAMID_INPAINT
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  bool initialize() final
  {
    return true;
  }

protected:
  DISPLAY _display = DISPLAY_RESTORED_IMAGE;
  enum INPAINT_METHOD _mask_inpaint_method = LINEAR_INTERPOLATION_INPAINT;
  double _S1_target = -1.2;
  double _macroStructSizePx = 150;
  //int _fftBorder = 0;
  bool _autoS1_target = true;
  bool _print_debug_info = false;
  bool _write_file = false;
  std::string _debug_file_name = "/home/projects/temp/analyze_profile.txt";


  // work arrays
  cv::Mat SRC_MASK;
  std::vector<cv::Mat> SRC_PLANES;
  std::vector<cv::Mat1f> SRC_P, SRC_S;
  cv::Mat1f RadialProfile;
  cv::Mat1f INVERSE_FILTER;
  std::vector<cv::Mat1f> SRC_CHANNELS_RESTORED;
  cv::Mat1f VLAP;
  int _prev_cn = 0;
};

#endif /* __c_fft_autosharp_routine_h__ */
