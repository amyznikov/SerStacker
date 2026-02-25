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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _eps = 0.1;
  int _level = 2;
};

#endif /* __c_normalize_mean_stdev_routine_h__ */
