/*
 * c_extract_channel_routine.h
 *
 *  Created on: Jan 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_extract_channel_routine_h__
#define __c_extract_channel_routine_h__

#include "c_image_processor.h"
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>

class c_extract_channel_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_extract_channel_routine, "extract_channel",
      "Calls <strong>extract_channel()</strong> to extract single color channel from image");

  void set_output_channel(enum color_channel_type v)
  {
    output_channel_ = v;
  }

  enum color_channel_type output_channel() const
  {
    return output_channel_;
  }

  void set_output_depth(enum PIXEL_DEPTH v)
  {
    output_depth_ = v;
  }

  enum PIXEL_DEPTH output_depth() const
  {
    return output_depth_;
  }

  void set_output_scale(double v)
  {
    output_scale_ = v;
  }

  double output_scale() const
  {
    return output_scale_;
  }

  void set_output_depth_scale(double v)
  {
    output_depth_scale_ = v;
  }

  double output_depth_scale() const
  {
    return output_depth_scale_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output_channel, "channel index or enum color_channel_type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output_depth, "depth");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output_scale, "optional output image scale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output_depth_scale, "optional output depth scale");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, output_channel);
      SERIALIZE_PROPERTY(settings, save, *this, output_depth);
      SERIALIZE_PROPERTY(settings, save, *this, output_scale);
      SERIALIZE_PROPERTY(settings, save, *this, output_depth_scale);

      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    return extract_channel(image.getMat(), image,
        mask.getMat(), mask,
        output_channel_,
        output_scale_,
        output_depth_,
        output_depth_scale_);
  }

protected:
  enum color_channel_type output_channel_ = color_channel_gray;
  enum PIXEL_DEPTH output_depth_ = PIXEL_DEPTH_NO_CHANGE;
  double output_scale_ = 1.;
  double output_depth_scale_ = 1.0;

};

#endif /* __c_extract_channel_routine_h__ */
