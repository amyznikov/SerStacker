/*
 * c_melp_pyramid_routine.h
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_melp_pyramid_routine_h__
#define __c_melp_pyramid_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/laplacian_pyramid.h>

class c_melp_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_melp_pyramid_routine,
      "melp", "Display MeLP Pyramid layers");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _minimum_image_size = 4;
  std::vector<int> _display_pos;
  c_melp_pyramid::sptr _pyramid;
};

#endif /* __c_melp_pyramid_routine_h__ */
