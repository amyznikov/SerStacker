/*
 * c_unsharp_mask_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_unsharp_mask_routine_h__
#define __c_unsharp_mask_routine_h__

#include <core/improc/c_image_processor.h>

class c_unsharp_mask_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_unsharp_mask_routine,
      "unsharp_mask", "Apply unsharp mask to image");

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
    sigma_ = v;
  }

  double sigma() const
  {
    return sigma_;
  }

  void set_alpha(double v)
  {
    alpha_ = v;
  }

  double alpha() const
  {
    return alpha_;
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

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, channel, "Color channel for sharpenning");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, alpha, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, blur_color_channels, "Gaussian blur sigma for color channels");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, outmin, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, outmax, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, channel);
      SERIALIZE_PROPERTY(settings, save, *this, sigma);
      SERIALIZE_PROPERTY(settings, save, *this, alpha);
      SERIALIZE_PROPERTY(settings, save, *this, blur_color_channels);
      SERIALIZE_PROPERTY(settings, save, *this, outmin);
      SERIALIZE_PROPERTY(settings, save, *this, outmax);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  double sigma_ = 1, alpha_ = 0.9;
  double outmin_ = -1, outmax_ = -1;
  COLOR_CHANNEL channel_ = COLOR_CHANNEL_ALL;
  double blur_color_channels_ = 0;
};

#endif /* __c_unsharp_mask_routine_h__ */
