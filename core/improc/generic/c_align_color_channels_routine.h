/*
 * c_align_color_channels_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_align_color_channels_routine_h__
#define __c_align_color_channels_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/image_registration/c_align_color_channels.h>

class c_align_color_channels_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_align_color_channels_routine,
      "align_color_channels", "Align color channels to reference one");

  void set_method(ECC_ALIGN_METHOD v)
  {
    _algorithm.set_method(v);
  }

  ECC_ALIGN_METHOD method() const
  {
    return _algorithm.method();
  }

  void set_reference_channel(int v)
  {
    _reference_channel = v;
  }

  int reference_channel() const
  {
    return _reference_channel;
  }

  void set_enable_threshold(bool v)
  {
    _enable_threshold = v;
  }

  bool enable_threshold() const
  {
    return _enable_threshold;
  }

  void set_threshold(double v)
  {
    _threshold = v;
  }

  double threshold() const
  {
    return _threshold;
  }

  void set_motion_type(IMAGE_MOTION_TYPE motion_type)
  {
    return _algorithm.set_motion_type(motion_type);
  }

  IMAGE_MOTION_TYPE motion_type() const
  {
    return _algorithm.motion_type();
  }

  void set_interpolation(enum ECC_INTERPOLATION_METHOD flags)
  {
    return _algorithm.set_interpolation(flags);
  }

  enum ECC_INTERPOLATION_METHOD interpolation() const
  {
    return _algorithm.interpolation();
  }

  void set_border_mode(enum ECC_BORDER_MODE border_mode)
  {
    _algorithm.set_border_mode(border_mode);
  }

  enum ECC_BORDER_MODE border_mode() const
  {
    return _algorithm.border_mode();
  }

  void set_border_value(const cv::Scalar & border_value)
  {
    _algorithm.set_border_value(border_value);
  }

  const cv::Scalar & border_value() const
  {
    return _algorithm.border_value();
  }

  void set_eps(double v)
  {
    _algorithm.set_eps(v);
  }

  double eps() const
  {
    return _algorithm.eps();
  }

  void set_max_level(int v)
  {
    _algorithm.set_max_level(v);
  }

  int max_level() const
  {
    return _algorithm.max_level();
  }

  void set_max_iterations(int v)
  {
    return _algorithm.set_max_iterations(v);
  }

  int max_iterations() const
  {
    return _algorithm.max_iterations();
  }

  void set_smooth_sigma(double v)
  {
    _algorithm.set_smooth_sigma(v);
  }

  double smooth_sigma() const
  {
    return _algorithm.smooth_sigma();
  }

  void set_update_step_scale(double v)
  {
    _algorithm.set_update_step_scale(v);
  }

  double update_step_scale() const
  {
    return _algorithm.update_step_scale();
  }

  void set_normalization_level(int v)
  {
    _algorithm.set_normalization_level(v);
  }

  int normalization_level() const
  {
    return _algorithm.normalization_level();
  }

  void set_normalization_eps(double v)
  {
    _algorithm.set_normalization_eps(v);
  }

  double normalization_eps() const
  {
    return _algorithm.normalization_eps();
  }

  c_align_color_channels & algorithm()
  {
    return _algorithm;
  }

  const c_align_color_channels & algorithm() const
  {
    return _algorithm;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_align_color_channels _algorithm;
  double _threshold = 0;
  int _reference_channel = 0;
  bool _enable_threshold = false;
};

#endif /* __c_align_color_channels_routine_h__ */
