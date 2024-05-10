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

  enum SharpenOrder {
    SharpenNone,
    SharpenBefore,
    SharpenAfter,
    SharpenEachIteration,
  };

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

  void set_sharpen_amount(double v)
  {
    sharpen_amount_ = v;
  }

  double sharpen_amount() const
  {
    return sharpen_amount_;
  }

  void set_sharpen_outmin(double v)
  {
    sharpen_outmin_ = v;
  }

  double sharpen_outmin() const
  {
    return sharpen_outmin_;
  }

  void set_sharpen_outmax(double v)
  {
    sharpen_outmax_ = v;
  }

  double sharpen_outmax() const
  {
    return sharpen_outmax_;
  }

  void set_sharpen_order(SharpenOrder v)
  {
    sharpen_order_ = v;
  }

  SharpenOrder sharpen_order() const
  {
    return sharpen_order_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  int count_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;
  SharpenOrder sharpen_order_ = SharpenNone;
  double sharpen_amount_ = 0.7;
  double sharpen_outmin_ = 0;
  double sharpen_outmax_ = 1e6;
};

#endif /* __c_pyrdown_routine_h__ */
