/*
 * c_normalize_mean_stdev_routine.h
 *
 *  Created on: Jul 2, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_normalize_mean_stdev_routine_h__
#define __c_normalize_mean_stdev_routine_h__

#include <core/improc/c_image_processor.h>

class c_normalize_mean_stdev_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_normalize_mean_stdev_routine,
      "normalize_mean_stdev",
      "Test for image normalization");

  void set_level(int v)
  {
    level_ = v;
  }

  int level() const
  {
    return level_;
  }

  void set_eps(double v)
  {
    eps_ = v;
  }

  double eps() const
  {
    return eps_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  double eps_ = 0.1;
  int level_ = 2;
};

#endif /* __c_normalize_mean_stdev_routine_h__ */
