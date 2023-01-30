/*
 * c_linear_interpolation_inpaint_routine.h
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_linear_interpolation_inpaint_routine_h__
#define __c_linear_interpolation_inpaint_routine_h__

#include "c_image_processor.h"
#include <core/proc/inpaint/linear_interpolation_inpaint.h>

class c_linear_interpolation_inpaint_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_linear_interpolation_inpaint_routine,
      "linear_interpolation_inpaint", "inpaint missing pixels in image");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if( !mask.empty() && mask.type() == CV_8UC1 ) {
      linear_interpolation_inpaint(image.getMat(), mask, image);
    }
    return true;
  }
};

#endif /* __c_linear_interpolation_inpaint_routine_h__ */
