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
    return &_options;
  }

  void set_octave(int v)
  {
    _octave = v;
  }

  int octave() const
  {
    return _octave;
  }

  void set_black_background(bool v)
  {
    _black_background = v;
  }

  bool black_background() const
  {
    return _black_background;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  void parameter_changed() final;

protected:
  c_feature2d::sptr _keypoints_detector;
  c_sparse_feature_detector_options _options;
  std::vector<cv::KeyPoint> _keypoints;
  cv::Mat _display;
  int _octave = -1;
  bool _black_background = false;
};

#endif /* __c_keypoins_detector_routine_h__ */
