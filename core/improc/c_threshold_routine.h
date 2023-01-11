/*
 * c_threshold_routine.h
 *
 *  Created on: Sep 18, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_threshold_routine_h__
#define __c_threshold_routine_h__

#include "c_image_processor.h"

enum THRESHOLD_TYPE {
  THRESHOLD_TYPE_CMP_GT,
  THRESHOLD_TYPE_CMP_LT,
  THRESHOLD_TYPE_CMP_GE,
  THRESHOLD_TYPE_CMP_LE,
  THRESHOLD_TYPE_CMP_EQ,
  THRESHOLD_TYPE_CMP_NE,
  THRESHOLD_TYPE_OTSU,
  THRESHOLD_TYPE_TRIANGLE,
  THRESHOLD_TYPE_MOMENTS,
  THRESHOLD_TYPE_ISODATA,
  THRESHOLD_TYPE_HUANG,
  THRESHOLD_TYPE_YEN,
  THRESHOLD_TYPE_MEAN,
  THRESHOLD_TYPE_MINIMUM
};

class c_threshold_routine:
    public c_image_processor_routine
{
public:
  typedef c_threshold_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("threshold", "threshold",
            "Uses <strong>cv::compare()</strong> or <strong>cv::threshold()</strong> to threshold image.<br>"
            "Each color channel is processed independently",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_threshold_routine(bool enabled = true);
  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_threshold_type(THRESHOLD_TYPE v);
  THRESHOLD_TYPE threshold_type() const;

  void set_threshold_value(double v);
  double threshold_value() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, threshold_type, "Threshold type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, threshold_value, "Threshold value");
  }

protected:
  double threshold_value_ = 0;
  THRESHOLD_TYPE threshold_type_ = THRESHOLD_TYPE_OTSU;
};

#endif /* __c_threshold_routine_h__ */
