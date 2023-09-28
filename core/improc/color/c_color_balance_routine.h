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

  void set_gscale(double v);
  double gscale() const;

  void set_alpha(double v);
  double alpha() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  double gscale_ = 1.0;
  double alpha_ = 0;
};

#endif /* __c_color_balance_routine_h__ */
