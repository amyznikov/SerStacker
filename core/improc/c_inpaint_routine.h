/*
 * c_inpaint_routine.h
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_inpaint_routine_h__
#define __c_inpaint_routine_h__

#include "c_image_processor.h"
#include <core/proc/inpaint.h>

class c_inpaint_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_inpaint_routine,
      "inpaint", "inpaint missing pixels in image");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    average_pyramid_inpaint(image.getMat(), mask, image);
    return true;
  }

};

#endif /* __c_inpaint_routine_h__ */
