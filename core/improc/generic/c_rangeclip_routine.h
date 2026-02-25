/*
 * c_rangeclip_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rangeclip_routine_h__
#define __c_rangeclip_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/autoclip.h>

class c_rangeclip_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_rangeclip_routine,
      "rangeclip", "Apply clip_range(min, max) for image");


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _min = 0.0;
  double _max = 1.0;
};

#endif /* __c_rangeclip_routine_h__ */
