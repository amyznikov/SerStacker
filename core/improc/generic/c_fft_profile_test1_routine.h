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
#include <core/proc/cdiffs.h>

class c_fft_profile_test1_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_profile_test1_routine,
      "fft_profile_test1", "c_fft_profile_test1_routine");

  enum DISPLAY {
    DISPLAY_SRC_IMAGE = 0,
    DISPLAY_SRC_MODULE,
    DISPLAY_SRC_PROFILE,

    DISPLAY_V_MATRIX,
    DISPLAY_V_MODULE,
    DISPLAY_VLAP_FILTER,
    DISPLAY_S_MATRIX,
    DISPLAY_S_MODULE,

//    DISPLAY_GAUSS_MODULE,
//    DISPLAY_GAUSS_PROFILE,
//    DISPLAY_GAUSS_FILTERED_MODULE,
//    DISPLAY_GAUSS_FILTERED_PROFILE,
//    DISPLAY_GAUSS_FILTERED_IMAGE,

//    DISPLAY_LAPL_MODULE,
//    DISPLAY_LAPL_PROFILE,
//    DISPLAY_LAPL_FILTERED_MODULE,
//    DISPLAY_LAPL_FILTERED_PROFILE,
//    DISPLAY_LAPL_FILTERED_IMAGE,

//    DISPLAY_GLAP_MODULE,
//    DISPLAY_GLAP_PROFILE,
//    DISPLAY_GLAP_FILTERED_MODULE,
//    DISPLAY_GLAP_FILTERED_PROFILE,
//    DISPLAY_GLAP_FILTERED_IMAGE,

    DISPLAY_FILTER,
    DISPLAY_RESTORED_MODULE,
    DISPLAY_RESTORED_IMAGE,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_SRC_IMAGE;
  bool _cleanSpectrum = false;
  bool _write_file = false;
};

#endif /* __c_fft_profile_test1_routine_h__ */
