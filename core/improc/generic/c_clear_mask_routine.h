/*
 * c_clear_mask_routine.h
 *
 *  Created on: Apr 4, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_clear_mask_routine_h__
#define __c_clear_mask_routine_h__

#include <core/improc/c_image_processor.h>

class c_clear_mask_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_clear_mask_routine,
      "clear_mask", "Release current image mask)");

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
};

#endif /* __c_clear_mask_routine_h__ */
