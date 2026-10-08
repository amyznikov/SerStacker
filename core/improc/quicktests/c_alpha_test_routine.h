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

class c_alpha_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_alpha_test_routine,
      "alpha_test", "Alpha Test");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected: // Controlling parameters
  //c_estimate_image_transform_options opts;
  c_sparse_feature_detector_options opts1;
  c_sparse_feature_descriptor_options opts2;
};

#endif /* __c_alpha_test_routine_h__ */
