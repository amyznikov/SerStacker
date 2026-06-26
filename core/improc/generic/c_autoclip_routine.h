/*
 * c_autoclip_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_autoclip_routine_h__
#define __c_autoclip_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_autoclip_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_autoclip_routine,
      "autoclip_routine", "Auto clip image histogram");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  PIXEL_DEPTH _ddepth = PIXEL_DEPTH_32F;
  cv::Vec2d _clipRange = cv::Vec2d(1, 99.99);
  cv::Vec2d _outputRange = cv::Vec2d(0, 1);
  bool _autoOutputRange = true;
};


#endif /* __c_autoclip_routine_h__ */
