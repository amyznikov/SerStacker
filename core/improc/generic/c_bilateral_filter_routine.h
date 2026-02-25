/*
 * c_bilateral_filter_routine.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_bilateral_filter_routine_h__
#define __c_bilateral_filter_routine_h__

#include <core/improc/c_image_processor.h>

class c_bilateral_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_bilateral_filter_routine,
      "bilateral", "Apply cv::bilateralFilter()");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _d = 0;
  double _sigmaColor = 1;
  double _sigmaSpace = 1;
  cv::BorderTypes _borderType = cv::BORDER_DEFAULT;
};

#endif /* __c_bilateral_filter_routine_h__ */
