/*
 * c_wmf_routine.h
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_wmf_routine_h__
#define __c_wmf_routine_h__

#include <core/improc/c_image_processor.h>
#include <opencv2/ximgproc.hpp>

class c_wmf_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_wmf_routine,
      "wmf",
      "calls cv::ximgproc::weightedMedianFilter()");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::ximgproc::WMFWeightType _weightType = cv::ximgproc::WMF_OFF;
  int _radius = 1;
  double _sigma = 25.5 / 255;
};

#endif /* __c_wmf_routine_h__ */
