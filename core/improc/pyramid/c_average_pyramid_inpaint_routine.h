/*
 * c_average_pyramid_inpaint_routine.h
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_average_pyramid_inpaint_routine_h__
#define __c_average_pyramid_inpaint_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/inpaint.h>

class c_average_pyramid_inpaint_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_average_pyramid_inpaint_routine,
      "average_pyramid_inpaint", "inpaint missing pixels in image");

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  int max_levels = 1;
};

#endif /* __c_average_pyramid_inpaint_routine_h__ */
