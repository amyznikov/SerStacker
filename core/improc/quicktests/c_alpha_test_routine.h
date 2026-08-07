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
    DISPLAY_G0,
//    DISPLAY_DIFF,
//    DISPLAY_SUMM,
//    DISPLAY_RATIO,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_SRC;
  enum color_channel_type _intensity_channel = color_channel_gray;
  int _l0 = 1;
  int _l2 = 6;
  int _kradius0 = 3;
  int _kradius1 = 6;
};

#endif /* __c_alpha_test_routine_h__ */
