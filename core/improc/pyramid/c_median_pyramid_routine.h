/*
 * c_median_pyramid_routine.h
 *
 *  Created on: Sep 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_pyramid_routine_h__
#define __c_median_pyramid_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/median_pyramid.h>

class c_median_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_pyramid_routine,
      "median_pyramid", "Display Median Pyramid layers");

  enum DisplayType {
    DisplayMedianBlur,
    DisplayMedianHat,
    DisplayScaledImage,
  };

  void set_display_type(DisplayType v)
  {
    display_type_ = v;
  }

  DisplayType display_type() const
  {
    return display_type_;
  }

  void set_display_level(int v)
  {
    display_level_ = v;
  }

  int display_level() const
  {
    return display_level_;
  }

  void set_ksize(int v)
  {
    ksize_  = v;
  }

  int ksize()
  {
    return ksize_;
  }

  void set_median_iterations(int v)
  {
    median_iterations_ = v;
  }

  int median_iterations() const
  {
    return median_iterations_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  DisplayType display_type_ = DisplayMedianBlur;
  int ksize_ = 3;
  int median_iterations_ = 1;
  int display_level_ = 0;
  std::vector<cv::Mat> scaled_images_;
  std::vector<cv::Mat> median_blurs_;
  std::vector<cv::Mat> median_hats_;
};

#endif /* __c_median_pyramid_routine_h__ */
