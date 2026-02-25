/*
 * c_birdview_transform_routine.h
 *
 *  Created on: Jul 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_birdview_transform_routine_h__
#define __c_birdview_transform_routine_h__

#include <core/improc/c_image_processor.h>

class c_birdview_transform_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_birdview_transform_routine,
      "birdview_transform", "Apply birdview_homography to image");

  void set_output_image_size(const cv::Size & v)
  {
    _output_image_size = v;
    H.release();
  }

  const cv::Size & output_image_size() const
  {
    return _output_image_size;
  }

  void set_src_line(double v)
  {
    _src_line = v;
    H.release();
  }

  double src_line() const
  {
    return _src_line;
  }

  void set_stretch(double v)
  {
    _stretch = v;
    H.release();
  }

  double stretch() const
  {
    return _stretch;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Mat H; // 3x3 Homography Matrix
  cv::Size _input_image_size;
  cv::Size _output_image_size;
  double _stretch = 3;
  double _src_line = -1;
};

#endif /* __c_birdview_transform_routine_h__ */
