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
    algorithm_.set_method(v);
  }

  ECC_ALIGN_METHOD method() const
  {
    return algorithm_.method();
  }

  void set_reference_channel(int v)
  {
    reference_channel_ = v;
  }

  int reference_channel() const
  {
    return reference_channel_;
  }

  void set_enable_threshold(bool v)
  {
    enable_threshold_ = v;
  }

  bool enable_threshold() const
  {
    return enable_threshold_;
  }

  void set_threshold(double v)
  {
    threshold_ = v;
  }

  double threshold() const
  {
    return threshold_;
  }

  void set_motion_type(IMAGE_MOTION_TYPE motion_type)
  {
    return algorithm_.set_motion_type(motion_type);
  }

  IMAGE_MOTION_TYPE motion_type() const
  {
    return algorithm_.motion_type();
  }

  void set_interpolation(enum ECC_INTERPOLATION_METHOD flags)
  {
    return algorithm_.set_interpolation(flags);
  }

  enum ECC_INTERPOLATION_METHOD interpolation() const
  {
    return algorithm_.interpolation();
  }

  void set_border_mode(enum ECC_BORDER_MODE border_mode)
  {
    algorithm_.set_border_mode(border_mode);
  }

  enum ECC_BORDER_MODE border_mode() const
  {
    return algorithm_.border_mode();
  }

  void set_border_value(const cv::Scalar & border_value)
  {
    algorithm_.set_border_value(border_value);
  }

  const cv::Scalar & border_value() const
  {
    return algorithm_.border_value();
  }

  void set_eps(double v)
  {
    algorithm_.set_eps(v);
  }

  double eps() const
  {
    return algorithm_.eps();
  }

  void set_max_level(int v)
  {
    algorithm_.set_max_level(v);
  }

  int max_level() const
  {
    return algorithm_.max_level();
  }

  void set_max_iterations(int v)
  {
    return algorithm_.set_max_iterations(v);
  }

  int max_iterations() const
  {
    return algorithm_.max_iterations();
  }

  void set_smooth_sigma(double v)
  {
    algorithm_.set_smooth_sigma(v);
  }

  double smooth_sigma() const
  {
    return algorithm_.smooth_sigma();
  }

  void set_update_step_scale(double v)
  {
    algorithm_.set_update_step_scale(v);
  }

  double update_step_scale() const
  {
    return algorithm_.update_step_scale();
  }

  void set_normalization_level(int v)
  {
    algorithm_.set_normalization_level(v);
  }

  int normalization_level() const
  {
    return algorithm_.normalization_level();
  }

  void set_normalization_eps(double v)
  {
    algorithm_.set_normalization_eps(v);
  }

  double normalization_eps() const
  {
    return algorithm_.normalization_eps();
  }

  c_align_color_channels & algorithm()
  {
    return algorithm_;
  }

  const c_align_color_channels & algorithm() const
  {
    return algorithm_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;


protected:
  c_align_color_channels algorithm_;
  int reference_channel_ = 0;
  bool enable_threshold_ = false;
  double threshold_ = 0;
};

#endif /* __c_align_color_channels_routine_h__ */
