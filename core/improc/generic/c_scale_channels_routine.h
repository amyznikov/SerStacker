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
    return bias_;
  }

  void set_bias(const cv::Scalar & v)
  {
    bias_ = v;
  }

  double bias_r() const
  {
    return bias_[2];
  }

  void set_bias_r(double v)
  {
    bias_[2] = v;
  }

  double bias_g() const
  {
    return bias_[1];
  }

  void set_bias_g(double v)
  {
    bias_[1] = v;
  }

  double bias_b() const
  {
    return bias_[0];
  }

  void set_bias_b(double v)
  {
    bias_[0] = v;
  }

  double bias_a() const
  {
    return bias_[3];
  }

  void set_bias_a(double v)
  {
    bias_[3] = v;
  }

  const cv::Scalar & stretch() const
  {
    return stretch_;
  }

  void set_stretch(const cv::Scalar & v)
  {
    stretch_ = v;
  }

  double stretch_r() const
  {
    return stretch_[2];
  }

  void set_stretch_r(double v)
  {
    stretch_[2] = v;
  }

  double stretch_g() const
  {
    return stretch_[1];
  }

  void set_stretch_g(double v)
  {
    stretch_[1] = v;
  }

  double stretch_b() const
  {
    return stretch_[0];
  }

  void set_stretch_b(double v)
  {
    stretch_[0] = v;
  }

  double stretch_a() const
  {
    return stretch_[3];
  }

  void set_stretch_a(double v)
  {
    stretch_[3] = v;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, stretch_r, "");
    BIND_PCTRL(ctls, bias_r, "");
    BIND_PCTRL(ctls, stretch_g, "");
    BIND_PCTRL(ctls, bias_g, "");
    BIND_PCTRL(ctls, stretch_b, "");
    BIND_PCTRL(ctls, bias_b, "");
    BIND_PCTRL(ctls, stretch_a, "");
    BIND_PCTRL(ctls, bias_a, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, bias);
      SERIALIZE_PROPERTY(settings, save, *this, stretch);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;



protected:
  cv::Scalar bias_ = cv::Scalar::all(0);
  cv::Scalar stretch_ = cv::Scalar::all(1);
};

#endif /* __c_scale_channels_routine_h__ */
