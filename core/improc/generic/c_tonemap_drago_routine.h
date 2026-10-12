/*
 * c_tonemap_drago_routine.h
 *
 *  Created on: Oct 11, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_tonemap_drago_routine__
#define __c_tonemap_drago_routine__

#include <core/improc/c_image_processor.h>

class c_tonemap_drago_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_tonemap_drago_routine,
      "tonemap_drago", "Apply cv::TonemapDrago");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
protected:
  cv::Ptr<cv::TonemapDrago> tonemap;
  float gamma = 1.0f;
  float saturation = 1.0f;
  float bias = 0.85f;

  float old_gamma = 1.0f;
  float old_saturation = 1.0f;
  float old_bias = 0.85f;


};

#endif /* __c_tonemap_drago_routine__ */
