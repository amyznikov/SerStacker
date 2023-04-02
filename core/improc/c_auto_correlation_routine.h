/*
 * c_auto_correlation_routine.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_auto_correlation_routine_h__
#define __c_auto_correlation_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/fft.h>

class c_auto_correlation_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_auto_correlation_routine,
      "auto_correlation", "Compute image auto correlation matrix");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    fftComputeAutoCorrelation(image.getMat(), image.getMatRef(), true);
    return true;
  }
};

#endif /* __c_auto_correlation_routine_h__ */
