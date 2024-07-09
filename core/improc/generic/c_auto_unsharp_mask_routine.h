/*
 * c_auto_unsharp_mask_routine.h
 *
 *  Created on: Sep 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_auto_unsharp_mask_routine_h__
#define __c_auto_unsharp_mask_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/focus.h>

class c_auto_unsharp_mask_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_auto_unsharp_mask_routine,
      "auto_unsharp_mask", "Apply auto unsharp mask to image");

  enum COLOR_CHANNEL {
    COLOR_CHANNEL_ALL = -1,
    COLOR_CHANNEL_0 = 0,
    COLOR_CHANNEL_1 = 1,
    COLOR_CHANNEL_2 = 2,
    COLOR_CHANNEL_3 = 3,
    COLOR_CHANNEL_YCrCb = 4,
    COLOR_CHANNEL_Lab = 5,
    COLOR_CHANNEL_Luv = 6,
    COLOR_CHANNEL_HSV = 7,
    COLOR_CHANNEL_HLS = 8,
  };


  void set_channel(COLOR_CHANNEL v)
  {
    channel_ = v;
  }

  COLOR_CHANNEL channel() const
  {
    return channel_;
  }

  void set_sigma(double v)
  {
    measure_.set_sigma(v);
  }

  double sigma() const
  {
    return measure_.sigma();
  }

  void set_target_sharpness(double v)
  {
    target_sharpness_ = v;
  }

  double target_sharpness() const
  {
    return target_sharpness_;
  }

  void set_norm_type(cv::NormTypes v)
  {
    measure_.set_norm_type(v);
  }

  cv::NormTypes norm_type() const
  {
    return measure_.norm_type();
  }

  void set_alpha_factor(double v)
  {
    alpha_factor_ = v;
  }

  double alpha_factor() const
  {
    return alpha_factor_;
  }

  void set_blur_color_channels(double v)
  {
    blur_color_channels_ = v;
  }

  double blur_color_channels() const
  {
    return blur_color_channels_;
  }

  void set_outmin(double v)
  {
    outmin_ = v;
  }

  double outmin() const
  {
    return outmin_;
  }

  void set_outmax(double v)
  {
    outmax_ = v;
  }

  double outmax() const
  {
    return outmax_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, channel, "Color channel for sharpenning");
    BIND_PCTRL(ctls, sigma, "");
    BIND_PCTRL(ctls, target_sharpness, "");
    BIND_PCTRL(ctls, alpha_factor, "");
    BIND_PCTRL(ctls, norm_type, "");
    BIND_PCTRL(ctls, blur_color_channels, "Gaussian blur sigma for color channels");
    BIND_PCTRL(ctls, outmin, "");
    BIND_PCTRL(ctls, outmax, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, channel);
      SERIALIZE_PROPERTY(settings, save, *this, sigma);
      SERIALIZE_PROPERTY(settings, save, *this, target_sharpness);
      SERIALIZE_PROPERTY(settings, save, *this, alpha_factor);
      SERIALIZE_PROPERTY(settings, save, *this, norm_type);
      SERIALIZE_PROPERTY(settings, save, *this, blur_color_channels);
      SERIALIZE_PROPERTY(settings, save, *this, outmin);
      SERIALIZE_PROPERTY(settings, save, *this, outmax);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  c_sharpness_norm_measure measure_;
  double target_sharpness_ = 0.5;
  double alpha_factor_ = 1.5;
  double outmin_ = -1, outmax_ = -1;
  double blur_color_channels_ = 0;
  COLOR_CHANNEL channel_ = COLOR_CHANNEL_ALL;
};


#endif /* __c_auto_unsharp_mask_routine_h__ */
