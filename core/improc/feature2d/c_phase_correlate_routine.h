/*
 * c_phase_correlate_routine.h
 *
 *  Created on: Sep 8, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_phase_correlate_routine_h__
#define __c_phase_correlate_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/image_registration/c_phase_correlate.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>

class c_phase_correlate_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_phase_correlate_routine,
      "c_phase_correlate", "c_phase_correlate tester");

  enum DISPLAY {
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_REFERENCE_IMAGE,
    DISPLAY_CURRENT_SCALED_IMAGE,
    DISPLAY_REFERENCE_SCALED_IMAGE,
    DISPLAY_IFFT,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Mat referenceImage, referenceMask;
  DISPLAY _display = DISPLAY_CURRENT_IMAGE;
  bool _fillMaskHoles = false;
  bool _updateReferenceImage = true;

  c_phase_correlate_options opts;
  c_phase_correlate pc;
};

#endif /* __c_phase_correlate_routine_h__ */
