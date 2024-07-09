/*
 * c_hessian_routine.h
 *
 *  Created on: May 31, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hessian_routine_h__
#define __c_hessian_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_hessian_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_hessian_routine,
      "hessian", "c_hessian_routine");

  enum OutputType {
    OutputGxx,
    OutputGyy,
    OutputGxy,
    OutputDet,
    OutputMaxEigenValues,
    OutputMinEigenValues,
  };

  void set_output_type(OutputType v)
  {
    output_type_ = v;
  }

  OutputType output_type() const
  {
    return output_type_;
  }

  void set_border_type(cv::BorderTypes v)
  {
    border_type_ = v;
  }

  cv::BorderTypes border_type() const
  {
    return border_type_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  OutputType output_type_ = OutputGxx;
  cv::BorderTypes border_type_  = cv::BORDER_DEFAULT;
};

#endif /* __c_hessian_routine_h__ */
