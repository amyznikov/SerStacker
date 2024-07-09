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
    THRESHOLD_CLEAR
  };

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

  void set_threshold_scale(double v)
  {
    threshold_scale_ = v;
  }

  double threshold_scale() const
  {
    return threshold_scale_;
  }

  void set_fill_holes(bool v)
  {
    fill_holes_ = v;
  }

  bool fill_holes() const
  {
    return fill_holes_;
  }

  void set_invert(bool v)
  {
    invert_ = v;
  }

  bool invert() const
  {
    return invert_;
  }

  void set_modify_mask(bool v)
  {
    modify_mask_ = v;
  }

  bool modify_mask() const
  {
    return modify_mask_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, compare, "Compare operation");
    BIND_PCTRL(ctls, threshold_type, "Threshold type");
    BIND_PCTRL(ctls, threshold_value, "Threshold value");
    BIND_PCTRL(ctls, threshold_scale, "Threshold scale");
    BIND_PCTRL(ctls, fill_holes, "fill_holes");
    BIND_PCTRL(ctls, invert, "invert");
    BIND_PCTRL(ctls, modify_mask, "Modify mask instead of image");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, compare);
      SERIALIZE_PROPERTY(settings, save, *this, threshold_type);
      SERIALIZE_PROPERTY(settings, save, *this, threshold_value);
      SERIALIZE_PROPERTY(settings, save, *this, threshold_scale);
      SERIALIZE_PROPERTY(settings, save, *this, fill_holes);
      SERIALIZE_PROPERTY(settings, save, *this, invert);
      SERIALIZE_PROPERTY(settings, save, *this, modify_mask);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::CmpTypes compare_ = cv::CMP_GT;
  THRESHOLD_TYPE threshold_type_ = THRESHOLD_VALUE;
  double threshold_value_ = 0;
  double threshold_scale_ = 1.0;
  bool fill_holes_ = false;
  bool invert_ = false;
  bool modify_mask_ = true;
};

#endif /* __c_threshold_routine_h__ */
