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
#include <core/proc/image_registration/c_phase_correlate.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>
#include <core/proc/image_registration/image_transform.h>


class c_alpha_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_alpha_test_routine,
      "alpha_test", "Alpha Test");

  enum DISPLAY {
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_DRIZZLED_IMAGE,
    DISPLAY_DRIZZLE_ACCUMULATOR,
    DISPLAY_DRIZZLE_WEIGHTS,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected: // Controlling parameters
  DISPLAY _display = DISPLAY_CURRENT_IMAGE;
  double _drizzleScale = 1.5;
  double _drizzlePixFrac = 0.6;

  struct c_translation_opts {
    cv::Vec2d T;
  } translation;
};

#endif /* __c_alpha_test_routine_h__ */
