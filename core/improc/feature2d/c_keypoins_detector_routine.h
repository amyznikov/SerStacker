/*
 * c_keypoins_detector_routine.h
 *
 *  Created on: Aug 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_keypoins_detector_routine_h__
#define __c_keypoins_detector_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/feature2d/feature2d.h>

class c_keypoins_detector_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_keypoins_detector_routine,
      "keypoins_detector", "Extract and draw feature2d keypoints on image");

  c_sparse_feature_detector_options * options()
  {
    return &options_;
  }

  void set_black_background(bool v)
  {
    black_background_ = v;
  }

  bool black_background() const
  {
    return black_background_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;
  void parameter_changed() override;

protected:
  c_feature2d::sptr keypoints_detector_;
  c_sparse_feature_detector_options options_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat display_;
  bool black_background_ = false;
};

#endif /* __c_keypoins_detector_routine_h__ */
