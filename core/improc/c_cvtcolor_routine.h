/*
 * c_cvtcolor_routine.h
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cvtcolor_routine_h__
#define __c_cvtcolor_routine_h__

#include "c_image_processor.h"

class c_cvtcolor_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_cvtcolor_routine,
      "cvtcolor_routine", "Calls cv::cvtColor()");

  void set_conversion(enum cv::ColorConversionCodes v)
  {
    code_ = v;
  }

  enum cv::ColorConversionCodes conversion() const
  {
    return code_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, conversion, "enum cv::ColorConversionCodes");
  }

  bool deserialize(c_config_setting settings)
  {
    if ( !base::deserialize(settings) ) {
      return false;
    }

    LOAD_PROPERTY(settings, *this, conversion);

    return true;
  }

  bool serialize(c_config_setting settings) const
  {
    if ( !base::serialize(settings) ) {
      return false;
    }

    SAVE_PROPERTY(settings, *this, conversion);

    return true;
  }


  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    if ( code_ != cv::COLOR_COLORCVT_MAX ) {
      cv::cvtColor(image.getMat(), image, code_);
    }
    return true;
  }

protected:
  enum cv::ColorConversionCodes code_ =
      cv::COLOR_COLORCVT_MAX;
};


#endif /* __c_cvtcolor_routine_h__ */
