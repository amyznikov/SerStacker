/*
 * c_tangential_transform_routine.h
 *
 *  Created on: Jul 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_tangential_transform_routine_h__
#define __c_tangential_transform_routine_h__

#include <core/improc/c_image_processor.h>

class c_tangential_transform_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_tangential_transform_routine,
      "tangential_transform", "Apply tangential transform to image");

  void set_focus(double v)
  {
    _focus = v;
    _rmap.release();
  }

  double focus() const
  {
    return _focus;
  }

  void set_center(const cv::Point2d & v)
  {
    _center = v;
    _rmap.release();
  }

  const cv::Point2d & center() const
  {
    return _center;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _focus = 1000;
  cv::Point2d _center = cv::Point2d(-1, -1);
  cv::Mat2f _rmap;
  cv::Size _previous_image_size;
};

#endif /* __c_tangential_transform_routine_h__ */
