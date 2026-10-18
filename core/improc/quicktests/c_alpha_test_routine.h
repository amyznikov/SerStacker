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
#include <core/proc/image_registration/estimate_image_transform.h>
#include <core/proc/feature2d/feature_extraction.h>
#include <core/proc/feature2d/feature2d_settings.h>
#include <core/proc/mreduce.h>

class c_alpha_test_routine :
    public c_image_processor_routine
{
public:
  enum DISPLAY {
    DISPLAY_SRC1,
    DISPLAY_SRC2,
    DISPLAY_SPEC1,
    DISPLAY_SPEC2,
    DISPLAY_SPEC3,
    DISPLAY_CROSS_SPEC13_DIFF,
    DISPLAY_CROSS_SPEC13,
    DISPLAY_CORR_MAP13,
  };

  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_alpha_test_routine,
      "alpha_test", "Alpha Test");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected: // Controlling parameters
  DISPLAY _display = DISPLAY_SRC2;
  cv::Mat1f VLAP, BPF, APW;
  cv::Vec2f _translation;
  double _angle = 0; // rotation ange
  double _sigma = 3; // bandpass filter sigma
  double _oldsigma = 3;
  bool _whitenSpecs = true;
};

#endif /* __c_alpha_test_routine_h__ */
