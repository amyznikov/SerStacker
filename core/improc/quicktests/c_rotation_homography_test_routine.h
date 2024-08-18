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

  void set_rotation(const cv::Vec3f & v)
  {
    _A = v;
  }

  const cv::Vec3f& rotation() const
  {
    return _A;
  }

  void set_center(const cv::Vec2f & v)
  {
    _C = v;
  }

  const cv::Vec2f & center() const
  {
    return _C;
  }

  void set_focus(float v)
  {
    _F = v;
  }

  float focus() const
  {
    return _F;
  }

  void set_output_size(const cv::Size & v)
  {
    _output_size = v;
  }

  const cv::Size & output_size() const
  {
    return _output_size;
  }

  void set_inverse_remap(bool v)
  {
    _inverse_remap = v;
  }

  bool inverse_remap() const
  {
    return _inverse_remap;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  cv::Vec3f _A;
  cv::Vec2f _C = cv::Vec2f(500, 500);
  cv::Size _output_size;
  float _F = 1000;
  bool _inverse_remap = false;
};

#endif /* __c_rotation_homography_test_routine_h__ */
