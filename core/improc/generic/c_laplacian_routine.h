/*
 * c_laplacian_routine.h
 *
 *  Created on: Apr 5, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_laplacian_routine_h__
#define __c_laplacian_routine_h__

#include <core/improc/c_image_processor.h>

class c_laplacian_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_laplacian_routine,
      "laplacian", "Apply cv::filter2D() with 5x5 Laplacian kernel");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

};

#endif /* __c_laplacian_routine_h__ */
