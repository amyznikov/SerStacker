/*
 * c_fast_gaussian_blur_routine.h
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fast_gaussian_blur_routine_h__
#define __c_fast_gaussian_blur_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_gaussian_filter.h>
#include <core/proc/pixtype.h>

class c_fast_gaussian_blur_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fast_gaussian_blur_routine,
      "fast_gaussian_blur", "Gaussian Blur speedup based on Image Pyramid");

  void set_sigma(double v)
  {
    _sigma = v;
  }

  double sigma() const
  {
    return _sigma;
  }

  void set_border_type(cv::BorderTypes v)
  {
    _border_type = v;
  }

  cv::BorderTypes border_type() const
  {
    return _border_type;
  }

  void set_border_value(const cv::Scalar & v)
  {
    _border_value = v;
  }

  const cv::Scalar & border_value() const
  {
    return _border_value;
  }

  void set_ddepth(enum PIXEL_DEPTH v)
  {
    _ddepth = v;
  }

  enum PIXEL_DEPTH ddepth() const
  {
    return _ddepth;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  double _sigma = 1.0;
  PIXEL_DEPTH _ddepth = PIXEL_DEPTH_NO_CHANGE;
  cv::BorderTypes _border_type = cv::BORDER_REFLECT101;
  cv::Scalar _border_value = cv::Scalar::all(0);
};

#endif /* __c_fast_gaussian_blur_routine_h__ */
