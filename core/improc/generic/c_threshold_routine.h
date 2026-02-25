/*
 * c_threshold_routine.h
 *
 *  Created on: Sep 18, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_threshold_routine_h__
#define __c_threshold_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/threshold.h>

class c_threshold_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_threshold_routine,
      "threshold",
      "use of cv::compare() to threshold image. "
      "Each color channel is processed independently");

  enum THRESHOLD_TYPE {
    THRESHOLD_VALUE = THRESHOLD_TYPE_VALUE,
    THRESHOLD_OTSU  = THRESHOLD_TYPE_OTSU,
    THRESHOLD_TRIANGLE = THRESHOLD_TYPE_TRIANGLE,
    THRESHOLD_MOMENTS = THRESHOLD_TYPE_MOMENTS,
    THRESHOLD_ISODATA = THRESHOLD_TYPE_ISODATA,
    THRESHOLD_HUANG = THRESHOLD_TYPE_HUANG,
    THRESHOLD_YEN = THRESHOLD_TYPE_YEN,
    THRESHOLD_MEAN = THRESHOLD_TYPE_MEAN,
    THRESHOLD_MINIMUM = THRESHOLD_TYPE_MINIMUM,
    THRESHOLD_NOISE = THRESHOLD_TYPE_NOISE,
    THRESHOLD_PLANETARY_DISK,
    THRESHOLD_CLEAR_MASK
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::CmpTypes _compare = cv::CMP_GT;
  THRESHOLD_TYPE _threshold_type = THRESHOLD_VALUE;
  double _threshold_value = 0;
  double _threshold_scale = 1.0;
  bool _fill_holes = false;
  bool _invert = false;
};

#endif /* __c_threshold_routine_h__ */
