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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _scale_factor = 0.75;
  int _level = 1;
  cv::InterpolationFlags _interpolation = cv::INTER_AREA;
  DisplayType _display_type = DisplayImage;

};

#endif /* __c_resize_pyramid_routine_h__ */
