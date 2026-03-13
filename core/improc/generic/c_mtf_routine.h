/*
 * c_mtf_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mtf_routine_h__
#define __c_mtf_routine_h__

#include <core/mtf/mtf.h>
#include <core/improc/c_image_processor.h>

class c_mtf_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_mtf_routine,
      "mtf", "c_smooth_rational_mtf");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Vec2f inputRange = cv::Vec2f(-1.f,-1.f);
  cv::Vec2f outputRange = cv::Vec2f(-1.f,-1.f);
  double lclip = 0.0;
  double hclip = 1.0;
  double shadows = 0.0;
  double midtones = 0.5;
  double highlights = 0.0;
  c_mtf mtf;
};

#endif /* __c_mtf_routine_h__ */
