/*
 * c_remove_sharp_artifacts_routine.h
 *
 *  Created on: Jul 30, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_remove_sharp_artifacts_routine_h__
#define __c_remove_sharp_artifacts_routine_h__

#include <core/improc/c_image_processor.h>

class c_remove_sharp_artifacts_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_remove_sharp_artifacts_routine,
      "remove_sharp_artifacts",
      "Remove some artifacts which could be after applying unsharp mask");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Size _erode_radius = cv::Size(5, 5);
  double _noise_scale = 10;
  double _mask_blur_radius = 3;
  double _edge_blur_radius = 1.5;
  bool _fill_holes = true;
  bool _show_mask = false;
  bool _show_blured_image = false;
};

#endif /* __c_remove_sharp_artifacts_routine_h__ */
