/*
 * c_fft_profile_test1_routine.h
 *
 *  Created on: Jun 5, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_profile_test1_routine_h__
#define __c_fft_profile_test1_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>

class c_fft_profile_test1_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_profile_test1_routine,
      "fft_profile_test1", "c_fft_profile_test1_routine");

  enum DISPLAY {
    DISPLAY_SRC_IMAGE = 0,
    DISPLAY_RESTORED_IMAGE,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_SRC_IMAGE;
  enum color_channel_type _intensity_channel = color_channel_gray;
  bool _write_file = false;
};

#endif /* __c_fft_profile_test1_routine_h__ */
