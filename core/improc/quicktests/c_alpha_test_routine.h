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
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_PREVIOUS_IMAGE,
    DISPLAY_DCT_CURRENT,
    DISPLAY_DCT_PREVIOUS,
    DISPLAY_DCT_CROSS,
    DISPLAY_IDCT,
  };


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Mat prevImage, prevMask, RAMP;
  DISPLAY _display = DISPLAY_CURRENT_IMAGE;
  bool _updatePrevious = true;
  bool _applyDifferentiate = true;
  bool _applyRAMP = true;
  bool _applyMaskApodization = true;
  int _maskApodizationKernelRadius = 15;
};

#endif /* __c_alpha_test_routine_h__ */
