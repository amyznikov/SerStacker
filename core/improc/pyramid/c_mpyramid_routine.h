/*
 * c_mpyramid_routine.h
 *
 *  Created on: Aug 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mpyramid_routine_h__
#define __c_mpyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_mpyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_mpyramid_routine,
      "mpyramid", "Display Laplacian Magnitude pyramid layers");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _minimum_image_size = 4;
  int _display_pos = 0;
};

#endif /* __c_mpyramid_routine_h__ */
