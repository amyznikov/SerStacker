/*
 * c_laplacian_pyramid_routine.h
 *
 *  Created on: May 30, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_laplacian_pyramid_routine_h__
#define __c_laplacian_pyramid_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/laplacian_pyramid.h>

class c_laplacian_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_laplacian_pyramid_routine,
      "laplacian_pyramid", "Display Laplacian Pyramid layers");

  enum DisplayType {
    DisplayLaplacian,
    DisplayMean,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DisplayType _display_type = DisplayLaplacian;
  cv::BorderTypes _borderType = cv::BORDER_DEFAULT;
  int _minimum_image_size = 4;
  int _display_level = 1;
  bool _absdiff = false;
  std::vector<cv::Mat> lp_, mp_;
};

#endif /* __c_laplacian_pyramid_routine_h__ */
