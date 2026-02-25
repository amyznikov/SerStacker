/*
 * c_blur_test_routine.h
 *
 *  Created on: Jun 7, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_blur_test_routine_h__
#define __c_blur_test_routine_h__

#include <core/improc/c_image_processor.h>

class c_blur_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_blur_test_routine,
      "blur_test", "c_blur_test_routine");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _sigma = 1;
  double _w = 0.5;
};

#endif /* __c_blur_test_routine_h__ */
