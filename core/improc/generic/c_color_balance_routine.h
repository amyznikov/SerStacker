/*
 * c_color_balance_routine.h
 *
 *  Created on: Sep 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_color_balance_routine_h__
#define __c_color_balance_routine_h__

#include <core/improc/c_image_processor.h>

class c_color_balance_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_color_balance_routine,
      "color_balance", "Test for c_color_balance_routine");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _gscale = 1.0;
  double _alpha = 0;
};

#endif /* __c_color_balance_routine_h__ */
