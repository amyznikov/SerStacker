/*
 * c_amelp_pyramid_routine.h
 *
 *  Created on: May 6, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_amelp_pyramid_routine_h__
#define __c_amelp_pyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_amelp_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_amelp_pyramid_routine,
      "amelp_pyramid", "Display amelp pyramid layers");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::vector<cv::Mat> _pyramid;
  double _scale_factor = 0.75;
  int _display_pos = 0;
  int _max_level = 3;
};

#endif /* __c_amelp_pyramid_routine_h__ */
