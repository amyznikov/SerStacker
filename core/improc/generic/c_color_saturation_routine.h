/*
 * c_color_saturation_routine.h
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_color_saturation_routine_h__
#define __c_color_saturation_routine_h__

#include <core/improc/c_image_processor.h>


class c_color_saturation_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_color_saturation_routine,
      "color_saturation",
      "Multi-Scale color saturation.<br>"
      "Based on gimp_rgb_to_hsl() and OpenCV image pyramid.");


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::vector<double> _scales;
};

#endif /* __c_color_saturation_routine_h__ */
