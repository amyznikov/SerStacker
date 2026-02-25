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

  enum DisplayType {
    DisplayRichKeypoints,
    DisplayTestHomography
  };

  ////////////

  void set_display_type(DisplayType v)
  {
    _display_type = v;
  }

  DisplayType display_type() const
  {
    return _display_type;
  }

  ////////////

  c_sparse_feature_detector_options * options()
  {
    return &_opts;
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

  ////
  // Test Homography
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

  float focus() const
  {
    return F;
  }
  ////


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  void parameter_changed() final;
  void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_sparse_feature_detector_options _opts;
  c_feature2d::sptr _keypoints_detector;
  std::vector<cv::KeyPoint> _keypoints;
  cv::Mat _display;
  int _octave = -1;
  DisplayType _display_type = DisplayRichKeypoints;
  bool _black_background = false;

  ////
  // Test Homography
  cv::Vec3f A = cv::Vec3f(0, 0, 0);
  cv::Vec3f T = cv::Vec3f(0, 0, 1);
  float F = 1000;
  ////

};

#endif /* __c_keypoins_detector_routine_h__ */
