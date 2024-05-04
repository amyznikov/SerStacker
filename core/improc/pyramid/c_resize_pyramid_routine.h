/*
 * c_resize_pyramid_routine.h
 *
 *  Created on: May 3, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_resize_pyramid_routine_h__
#define __c_resize_pyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_resize_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_resize_pyramid_routine,
      "c_resize_pyramid", "Build image pyramid using cv;:resize()");

  enum DisplayType {
    DisplayImage,
    DisplayAbsdiff,
  };


  int level() const
  {
    return level_;
  }

  void set_level(int v)
  {
    level_ = v;
  }

  double scale_factor() const
  {
    return scale_factor_;
  }

  void set_scale_factor(double v)
  {
    scale_factor_ = v;
  }

  cv::InterpolationFlags interpolation() const
  {
    return interpolation_;
  }

  void set_interpolation(cv::InterpolationFlags v)
  {
    interpolation_ = v;
  }

  DisplayType display_type() const
  {
    return display_type_;
  }

  void set_display_type(DisplayType v)
  {
    display_type_ = v;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  double scale_factor_ = 0.75;
  int level_ = 1;
  cv::InterpolationFlags interpolation_ = cv::INTER_AREA;
  DisplayType display_type_ = DisplayImage;

};

#endif /* __c_resize_pyramid_routine_h__ */
