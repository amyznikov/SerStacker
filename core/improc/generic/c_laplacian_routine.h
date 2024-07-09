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
      "laplacian", "compute image laplacian");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;
};

#endif /* __c_laplacian_routine_h__ */
