/*
 * c_scale_channels_routine.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_scale_channels_routine_h__
#define __c_scale_channels_routine_h__

#include <core/improc/c_image_processor.h>

class c_scale_channels_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_scale_channels_routine,
      "scale_channels", "stretch color channel histograms");

  const cv::Scalar & bias() const
  {
    return _bias;
  }

  void set_bias(const cv::Scalar & v)
  {
    _bias = v;
  }

  double bias_r() const
  {
    return _bias[2];
  }

  void set_bias_r(double v)
  {
    _bias[2] = v;
  }

  double bias_g() const
  {
    return _bias[1];
  }

  void set_bias_g(double v)
  {
    _bias[1] = v;
  }

  double bias_b() const
  {
    return _bias[0];
  }

  void set_bias_b(double v)
  {
    _bias[0] = v;
  }

  double bias_a() const
  {
    return _bias[3];
  }

  void set_bias_a(double v)
  {
    _bias[3] = v;
  }

  const cv::Scalar & stretch() const
  {
    return _stretch;
  }

  void set_stretch(const cv::Scalar & v)
  {
    _stretch = v;
  }

  double stretch_r() const
  {
    return _stretch[2];
  }

  void set_stretch_r(double v)
  {
    _stretch[2] = v;
  }

  double stretch_g() const
  {
    return _stretch[1];
  }

  void set_stretch_g(double v)
  {
    _stretch[1] = v;
  }

  double stretch_b() const
  {
    return _stretch[0];
  }

  void set_stretch_b(double v)
  {
    _stretch[0] = v;
  }

  double stretch_a() const
  {
    return _stretch[3];
  }

  void set_stretch_a(double v)
  {
    _stretch[3] = v;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);


protected:
  cv::Scalar _bias = cv::Scalar::all(0);
  cv::Scalar _stretch = cv::Scalar::all(1);
};

#endif /* __c_scale_channels_routine_h__ */
