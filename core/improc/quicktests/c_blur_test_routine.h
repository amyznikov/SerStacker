/*
 * c_blur_test_routine.h
 *
 *  Created on: Jun 7, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_blur_test_routine_h__
#define __c_blur_test_routine_h__

#include <core/improc/c_image_processor.h>

class c_blur_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_blur_test_routine,
      "blur_test", "c_blur_test_routine");

  void set_sigma(double v)
  {
    sigma_ = v;
  }

  double sigma() const
  {
    return sigma_;
  }

  void set_w(double v)
  {
    w_ = v;
  }

  double w() const
  {
    return w_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  double sigma_ = 1;
  double w_ = 0.5;
};

#endif /* __c_blur_test_routine_h__ */
