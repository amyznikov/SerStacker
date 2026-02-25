/*
 * c_scale_gaussian_pyramid_layers_routine.h
 *
 *  Created on: Jul 13, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_gaussian_pyramid_routine_h__
#define __c_gaussian_pyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_scale_gaussian_pyramid_layers_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_scale_gaussian_pyramid_layers_routine,
      "scale_pyramid_layers", "Decompose image into gaussian pyramid, "
          "scale layers and recompose image back");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::vector<double> _scales;
  cv::BorderTypes _borderType = cv::BORDER_DEFAULT;
};

#endif /* __c_gaussian_pyramid_routine_h__ */
