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

  void set_conversion(enum cv::ColorConversionCodes v);
  enum cv::ColorConversionCodes conversion() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, conversion, "enum cv::ColorConversionCodes");
  }

protected:
  enum cv::ColorConversionCodes code_ =
      cv::COLOR_COLORCVT_MAX;
};


#endif /* __c_cvtcolor_routine_h__ */
