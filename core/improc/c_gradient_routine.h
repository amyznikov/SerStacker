/*
 * c_gradient_routine.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#ifndef __c_gradient_routine_h__
#define __c_gradient_routine_h__

#include "c_image_processor.h"

class c_gradient_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gradient_routine,
      "gradient", "compute image gradient magnitude");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;
};

#endif /* __c_gradient_routine_h__ */
