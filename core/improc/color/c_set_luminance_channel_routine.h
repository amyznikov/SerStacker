/*
 * c_set_luminance_channel_routine.h
 *
 *  Created on: May 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_set_luminance_channel_routine_h__
#define __c_set_luminance_channel_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/extract_channel.h>

class c_set_luminance_channel_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_set_luminance_channel_routine,
      "set_luminance_channel",
      "Set specified color channel as luminance.<br>"
      "For non-linear color spaces like Lab/Luv the input image must be normalized to standard range "
      "(e.g. [0..1] for CV_32F)");

  enum Colorspace {
    Colorspace_Lab,
    Colorspace_Luv,
    Colorspace_HSV,
    Colorspace_HLS,
    Colorspace_YCrCb,
  };

  void set_luminance_channel(enum color_channel_type v)
  {
    luminance_channel_ = v;
  }

  enum color_channel_type luminance_channel() const
  {
    return luminance_channel_;
  }

  void set_colorspace(enum Colorspace v)
  {
    colorspace_ = v;
  }

  enum Colorspace colorspace() const
  {
    return colorspace_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  enum color_channel_type luminance_channel_ = color_channel_red;
  enum Colorspace colorspace_ = Colorspace_Lab;
};

#endif /* __c_set_luminance_channel_routine_h__ */
