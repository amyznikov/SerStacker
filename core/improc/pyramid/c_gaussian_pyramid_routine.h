/*
 * c_gaussian_pyramid_routine.h
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pyrdown_routine_h__
#define __c_pyrdown_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/unsharp_mask.h>

class c_gaussian_pyramid_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gaussian_pyramid_routine, "gaussian_pyramid",
      "Calls <strong>cv::pyrDown()</strong> or <strong>cv::pyrUp()</strong> on image");

  void set_count(int v)
  {
    count_ = v;
  }

  int count() const
  {
    return count_;
  }

  void set_borderType(cv::BorderTypes v)
  {
    borderType_ = v;
  }

  cv::BorderTypes borderType() const
  {
    return borderType_;
  }

  void set_usharp_sigma(double v)
  {
    usharp_sigma_ = v;
  }

  double usharp_sigma() const
  {
    return usharp_sigma_;
  }

  void set_usharp_alpha(double v)
  {
    usharp_alpha_ = v;
  }

  double usharp_alpha() const
  {
    return usharp_alpha_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  int count_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;
  double usharp_sigma_ = 1.5;
  double usharp_alpha_ = 0.0;
};

#endif /* __c_pyrdown_routine_h__ */
