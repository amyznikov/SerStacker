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
    _reference_point = v;
    rmap_.release();
  }

  const cv::Point2f & reference_point() const
  {
    return _reference_point;
  }

  void set_disparity(int  v)
  {
    _disparity = v;
    rmap_.release();
  }

  int disparity() const
  {
    return _disparity;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Point2f _reference_point = cv::Point2f(0,0);
  int _disparity = 1;
  cv::Mat2f rmap_;
};

#endif /* __c_radial_scale_routine_h__ */
