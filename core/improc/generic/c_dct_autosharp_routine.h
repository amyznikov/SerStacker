/*
 * c_dct_autosharp_routine.h
 *
 *  Created on: Jul 23, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dct_autosharp_routine_h__
#define __c_dct_autosharp_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/extract_channel.h>

class c_dct_autosharp_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_dct_autosharp_routine,
      "dct_autosharp", "Auto sharpen raw stack with cv::dct()");

  enum DISPLAY {
    DISPLAY_SRC_IMAGE,
    DISPLAY_RESTORED_IMAGE,
    DISPLAY_FILL_SRC_VOIDS,
    DISPLAY_SRC_SPECTRUM,
    DISPLAY_SRC_RADIAL_PROFILE,
    DISPLAY_SRC_RADIAL_PROFILE_LOG,
    DISPLAY_FILTER,
    DISPLAY_RESTORED_SPECTRUM,
    DISPLAY_RESTORED_PROFILE,
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
  DISPLAY _display = DISPLAY_RESTORED_IMAGE;
  enum color_channel_type _intensity_channel = color_channel_gray;
  enum INPAINT_METHOD _mask_inpaint_method = LINEAR_INTERPOLATION_INPAINT;
  double _S1_target = -1.2;
  bool _useS1_target = true;
  bool _print_debug_info = false;
  bool _write_file = false;
  std::string _debug_file_name = "/home/projects/temp/analyze_profile.txt";
};

#endif /* __c_dct_autosharp_routine_h__ */
