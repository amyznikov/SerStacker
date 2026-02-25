/*
 * c_median_hat_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_hat_routine_h__
#define __c_median_hat_routine_h__

#include <core/improc/c_image_processor.h>

class c_median_hat_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_hat_routine,
      "median_hat",
      "Difference between input image and it's median blur");

  enum DisplayType {
    DisplayMedianBlur ,
    DisplayMedianHat ,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _radius = 2;
  int _iterations = 1;
  DisplayType _display_type = DisplayMedianHat;
  bool _absdiff = false;
};

#endif /* __c_median_hat_routine_h__ */
