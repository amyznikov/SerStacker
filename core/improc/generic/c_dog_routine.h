/*
 * c_dog_routine.h
 *
 *  Created on: Jul 31, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dog_routine_h__
#define __c_dog_routine_h__

#include <core/improc/c_image_processor.h>

class c_dog_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_dog_routine,
      "DoG", "Difference of Gaussians");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
protected:
  double sigma1 = 1;
  double sigma2 = 2;
  cv::BorderTypes borderType = cv::BORDER_REFLECT101;
  cv::Scalar borderValue = cv::Scalar::all(0);
};

#endif /* __c_dog_routine_h__ */
