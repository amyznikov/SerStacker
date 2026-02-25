/*
 * c_linear_interpolation_inpaint_routine.h
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_linear_interpolation_inpaint_routine_h__
#define __c_linear_interpolation_inpaint_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>

class c_linear_interpolation_inpaint_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_linear_interpolation_inpaint_routine,
      "linear_interpolation_inpaint", "inpaint missing pixels in image");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

};

#endif /* __c_linear_interpolation_inpaint_routine_h__ */
