/*
 * c_radial_scale_routine.h
 *
 *  Created on: Sep 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_radial_scale_routine_h__
#define __c_radial_scale_routine_h__

#include <core/improc/c_image_processor.h>

class c_radial_scale_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_radial_scale_routine,
      "radial_scale", "apply radial scale");

  void set_reference_point(const cv::Point2f & v)
  {
    reference_point_ = v;
    rmap_.release();
  }

  const cv::Point2f & reference_point() const
  {
    return reference_point_;
  }

  void set_disparity(int  v)
  {
    disparity_ = v;
    rmap_.release();
  }

  int disparity() const
  {
    return disparity_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Point2f reference_point_ = cv::Point2f(0,0);
  int disparity_ = 1;
  cv::Mat2f rmap_;
};

#endif /* __c_radial_scale_routine_h__ */
