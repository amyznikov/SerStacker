/*
 * c_rotation_homography_test_routine.h
 *
 *  Created on: Aug 17, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rotation_homography_test_routine_h__
#define __c_rotation_homography_test_routine_h__

#include <core/improc/c_image_processor.h>

class c_rotation_homography_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_rotation_homography_test_routine,
      "c_rotation_homography_test_routine", "test for pure rotation homography");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Vec3f _rotation;
  cv::Vec2f _center = cv::Vec2f(500, 500);
  cv::Size _output_size;
  float _focus = 1000;
  bool _inverse_remap = false;
};

#endif /* __c_rotation_homography_test_routine_h__ */
