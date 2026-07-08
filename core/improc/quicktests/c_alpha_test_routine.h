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
    DISPLAY_SRC_IMAGE,
    DISPLAY_BLUR1_IMAGE,
    DISPLAY_BLUR2_IMAGE,
    DISPLAY_TEXTURE_IMAGE,
    DISPLAY_BLURED_TEXTURE,
    DISPLAY_K_IMAGE,
    DISPLAY_FILTERED_IMAGE,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_FILTERED_IMAGE;
  //int _se_radius = 4;
  double _sigma1 = 0.8;
  double _sigma2 = 2;
  double _sigma_se = 1.5;
  double _lpgk = 0.2;
  double _tex_scale = 1;
};

#endif /* __c_alpha_test_routine_h__ */
