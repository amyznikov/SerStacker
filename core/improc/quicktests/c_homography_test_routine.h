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

  void set_rotation(const cv::Vec3f & v)
  {
    A = v;
  }

  const cv::Vec3f& rotation() const
  {
    return A;
  }

  void set_translation(const cv::Vec3f & v)
  {
    T = v;
  }

  const cv::Vec3f& translation() const
  {
    return T;
  }

  void set_focus(float v)
  {
    F = v;
  }

  void set_output_size(const cv::Size & v)
  {
    output_size_ = v;
  }

  const cv::Size & output_size() const
  {
    return output_size_;
  }

  float focus() const
  {
    return F;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Vec3f A;
  cv::Vec3f T;
  cv::Size output_size_;
  float F = 1000;
};

#endif /* __c_homography_test_routine_h__ */
