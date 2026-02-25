/*
 * c_image_processor_routine.h
 *
 *  Created on: Jul 15, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_blur_routine_h__
#define __c_median_blur_routine_h__

#include <core/improc/c_image_processor.h>
#include <opencv2/ximgproc.hpp>

class c_median_blur_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_blur_routine,
      "medianBlur",
      "Apply cv::medianBlur() to image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _radius = 1;
  int _iterations = 1;
};

#endif /* __c_median_blur_routine_h__ */
