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
      "Uses <strong>cv::compare()</strong> to threshold image. "
      "Each color channel is processed independently");

  void set_compare(cv::CmpTypes v)
  {
    compare_ = v;
  }

  cv::CmpTypes compare() const
  {
    return compare_;
  }

  void set_threshold_type(THRESHOLD_TYPE v)
  {
    threshold_type_ = v;
  }

  THRESHOLD_TYPE threshold_type() const
  {
    return threshold_type_;
  }

  void set_threshold_value(double v)
  {
    threshold_value_ = v;
  }

  double threshold_value() const
  {
    return threshold_value_;
  }

  void set_modify_mask(bool v)
  {
    modify_mask_ = v;
  }

  bool modify_mask() const
  {
    return modify_mask_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, compare, "Compare operation");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, threshold_type, "Threshold type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, threshold_value, "Threshold value");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, modify_mask, "Modify mask instead of image");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, compare);
      SERIALIZE_PROPERTY(settings, save, *this, threshold_type);
      SERIALIZE_PROPERTY(settings, save, *this, threshold_value);
      SERIALIZE_PROPERTY(settings, save, *this, modify_mask);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::CmpTypes compare_ = cv::CMP_GT;
  THRESHOLD_TYPE threshold_type_ = THRESHOLD_TYPE_VALUE;
  double threshold_value_ = 0;
  bool modify_mask_ = true;
};

#endif /* __c_threshold_routine_h__ */
