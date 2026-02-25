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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Size _dstize;
  double _fx = 0;
  double _fy = 0;
  cv::InterpolationFlags _interpolation = cv::INTER_AREA;
  cv::InterpolationFlags _mask_interpolation = cv::INTER_NEAREST;
  int _mask_threshold = 250;
};

#endif /* __c_resize_image_routine_h__ */
