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

  //
  void set_output_image_size(const cv::Size & v)
  {
    output_image_size_ = v;
    H.release();
  }

  const cv::Size & output_image_size() const
  {
    return output_image_size_;
  }

  //

  void set_src_line(double v)
  {
    src_line_ = v;
    H.release();
  }

  double src_line() const
  {
    return src_line_;
  }

  //
  void set_stretch(double v)
  {
    stretch_ = v;
    H.release();
  }

  double stretch() const
  {
    return stretch_;
  }

  //

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Mat H; // 3x3 Homography Matrix
  cv::Size input_image_size_;
  cv::Size output_image_size_;
  double stretch_ = 3;
  double src_line_ = -1;
};

#endif /* __c_birdview_transform_routine_h__ */
