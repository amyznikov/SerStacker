/*
 * c_absdiff_routine.h
 *
 *  Created on: Jan 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_absdiff_routine_h__
#define __c_absdiff_routine_h__

#include <core/improc/c_image_processor.h>

class c_absdiff_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_absdiff_routine,
      "absdiff", "calls cv::absdiff(image, value)");

  void set_value(const cv::Scalar & v)
  {
    value_ = v;
  }

  const cv::Scalar & value() const
  {
    return value_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, value, "cv::Scalar");
  }

  bool serialize(c_config_setting settings, bool save)
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, value);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override
  {
    cv::absdiff(image.getMat(), value_, image);
    return true;
  }

protected:
  cv::Scalar value_;
};

#endif /* __c_absdiff_routine_h__ */
