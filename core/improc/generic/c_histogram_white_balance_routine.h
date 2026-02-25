/*
 * c_histogram_white_balance_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_histogram_white_balance_routine_h__
#define __c_histogram_white_balance_routine_h__

#include <core/improc/c_image_processor.h>

class c_histogram_white_balance_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_histogram_white_balance_routine,
      "histogram_white_balance",
      "GrayWorld-like image white balance based on image histogram stretch");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _lclip = 1;
  double _hclip = 99;
  double _threshold = 0;
  bool _enable_threshold = false;
};

#endif /* __c_histogram_white_balance_routine_h__ */
