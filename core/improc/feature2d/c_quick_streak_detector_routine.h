/*
 * c_quick_streak_detector_routine.h
 *
 *  Created on: Nov 17, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_quick_streak_detector_routine_h__
#define __c_quick_streak_detector_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/feature2d/c_quick_streak_checker.h>

class c_quick_streak_detector_routine :
    public c_image_processor_routine
{
public:
  enum DISPLAY {
    DISPLAY_SRC,
    DISPLAY_TOPHAT,
    DISPLAY_TOPMASK,
  };

  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_quick_streak_detector_routine,
      "quick_streak_detector", "Check for satellite streaks on deep sky image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_TOPMASK;
  c_quick_streak_checker_options _opts;
  c_quick_streak_checker _streak_checker;
  bool _update_roi = false;
};

#endif /* __c_quick_streak_detector_routine_h__ */
