/*
 * c_noisemap_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_noisemap_routine_h__
#define __c_noisemap_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/estimate_noise.h>

class c_noisemap_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_noisemap_routine,
      "noisemap",
      " Estimate the standard deviation of the noise in a gray-scale image.<br>"
      "  J. Immerkr, Fast Noise Variance Estimation,<br>"
      "  Computer Vision and Image Understanding,<br>"
      "  Vol. 64, No. 2, pp. 300-302, Sep. 1996");

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

};



#endif /* __c_noisemap_routine_h__ */

