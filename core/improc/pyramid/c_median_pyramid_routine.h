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


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DisplayType _display_type = DisplayMedianBlur;
  int _ksize = 3;
  int _median_iterations = 1;
  int _display_level = 0;
  std::vector<cv::Mat> scaled_images_;
  std::vector<cv::Mat> median_blurs_;
  std::vector<cv::Mat> median_hats_;
};

#endif /* __c_median_pyramid_routine_h__ */
