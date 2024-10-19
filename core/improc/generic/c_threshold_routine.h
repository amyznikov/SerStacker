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

  void set_compare(cv::CmpTypes v)
  {
    _compare = v;
  }

  cv::CmpTypes compare() const
  {
    return _compare;
  }

  void set_threshold_type(THRESHOLD_TYPE v)
  {
    _threshold_type = v;
  }

  THRESHOLD_TYPE threshold_type() const
  {
    return _threshold_type;
  }

  void set_threshold_value(double v)
  {
    _threshold_value = v;
  }

  double threshold_value() const
  {
    return _threshold_value;
  }

  void set_threshold_scale(double v)
  {
    _threshold_scale = v;
  }

  double threshold_scale() const
  {
    return _threshold_scale;
  }

  void set_fill_holes(bool v)
  {
    _fill_holes = v;
  }

  bool fill_holes() const
  {
    return _fill_holes;
  }

  void set_invert(bool v)
  {
    _invert = v;
  }

  bool invert() const
  {
    return _invert;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  cv::CmpTypes _compare = cv::CMP_GT;
  THRESHOLD_TYPE _threshold_type = THRESHOLD_VALUE;
  double _threshold_value = 0;
  double _threshold_scale = 1.0;
  bool _fill_holes = false;
  bool _invert = false;
};

#endif /* __c_threshold_routine_h__ */
