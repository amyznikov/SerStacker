/*
 * c_alpha_test_routine.h
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_alpha_test_routine_h__
#define __c_alpha_test_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>

class c_alpha_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_alpha_test_routine,
      "alpha_test", "Alpha Test");

  enum DISPLAY {
    DISPLAY_SRC,
    DISPLAY_RESTORED_IMAGE,
    DISPLAY_FILL_SRC_VOIDS,
    DISPLAY_SRC_SPECTRUM,
    DISPLAY_RADIAL_PROFILE,
    DISPLAY_FILTER,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_SRC_SPECTRUM;
  enum color_channel_type _intensity_channel = color_channel_gray;
  int _maxH = 64;
  int _samplesPerH = 50000;
  std::string _debug_file_name = "/home/projects/temp/Variogram.txt";
};

#endif /* __c_alpha_test_routine_h__ */
