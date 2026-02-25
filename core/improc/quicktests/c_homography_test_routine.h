/*
 * c_homography_test_routine.h
 *
 *  Created on: Mar 6, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_homography_test_routine_h__
#define __c_homography_test_routine_h__

#include <core/improc/c_image_processor.h>

class c_homography_test_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_homography_test_routine,
      "homography_test", "homography_test");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Vec3f A;
  cv::Vec3f T;
  cv::Size _output_size;
  float F = 1000;
};

#endif /* __c_homography_test_routine_h__ */
