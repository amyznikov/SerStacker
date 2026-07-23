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
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  bool initialize() final
  {
    _anscombe.set_method(anscombe_none);
    return true;
  }

protected:
  DISPLAY _display = DISPLAY_RESTORED_IMAGE;
  enum color_channel_type _intensity_channel = color_channel_gray;
  double _S1_gain = 1;
  c_anscombe_transform _anscombe;
  std::string _debug_file_name = "/home/projects/temp/analyze_profile.txt";
  bool _write_file = false;


  // work arrays
  cv::Mat SRC_IMAGE;
  std::vector<cv::Mat2f> SRC_P, SRC_S;
  cv::Mat INTENSITY_CHANNEL;
  cv::Mat2f INTENSITY_P, INTENSITY_S;
  cv::Mat1f INTENSITY_Magnitude, INTENSITY_RadialProfile;
  std::vector<cv::Mat> SRC_CHANNELS_RESTORED;
  cv::Mat SRC_RESTORED;
  cv::Mat1f VLAP;
};

#endif /* __c_fft_autosharp_routine_h__ */
