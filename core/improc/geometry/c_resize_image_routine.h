/*
 * c_resize_image_routine.h
 *
 *  Created on: May 3, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_resize_image_routine_h__
#define __c_resize_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_resize_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_resize_image_routine,
      "c_resize_image", "Apply cv;:resize() to image");

  const cv::Size& dstize() const
  {
    return dstize_;
  }

  void set_dstize(const cv::Size & v)
  {
    dstize_ = v;
  }

  double fx() const
  {
    return fx_;
  }

  void set_fx(double v)
  {
    fx_ = v;
  }

  double fy() const
  {
    return fy_;
  }

  void set_fy(double v)
  {
    fy_ = v;
  }

  cv::InterpolationFlags interpolation() const
  {
    return interpolation_;
  }

  void set_interpolation(cv::InterpolationFlags v)
  {
    interpolation_ = v;
  }

  cv::InterpolationFlags mask_interpolation() const
  {
    return mask_interpolation_;
  }

  void set_mask_interpolation(cv::InterpolationFlags v)
  {
    mask_interpolation_ = v;
  }

  int mask_threshold() const
  {
    return mask_threshold_;
  }

  void set_mask_threshold(int v)
  {
    mask_threshold_ = v;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Size dstize_;
  double fx_ = 0;
  double fy_ = 0;
  cv::InterpolationFlags interpolation_ = cv::INTER_AREA;
  cv::InterpolationFlags mask_interpolation_ = cv::INTER_NEAREST;
  int mask_threshold_ = 250;
};

#endif /* __c_resize_image_routine_h__ */
