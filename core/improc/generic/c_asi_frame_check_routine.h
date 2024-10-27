/*
 * c_asi_frame_check_routine.h
 *
 *  Created on: Oct 22, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_asi_frame_check_routine_h__
#define __c_asi_frame_check_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/io/debayer.h>

class c_asi_frame_check_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_asi_frame_check_routine,
      "asi_frame_check", "Checi ASI frame integrity");

  enum BAYER_PATTERN {
    BAYER_RGGB = COLORID_BAYER_RGGB,
    BAYER_GRBG = COLORID_BAYER_GRBG,
    BAYER_GBRG = COLORID_BAYER_GBRG,
    BAYER_BGGR = COLORID_BAYER_BGGR,
    BAYER_CYYM = COLORID_BAYER_CYYM,
    BAYER_YCMY = COLORID_BAYER_YCMY,
    BAYER_YMCY = COLORID_BAYER_YMCY,
    BAYER_MYYC = COLORID_BAYER_MYYC,
  };

  enum DISPLAY_CHANNEL {
    DISPLAY_CHANNEL_0 = 0,
    DISPLAY_CHANNEL_1 = 1,
    DISPLAY_CHANNEL_2 = 2,
    DISPLAY_CHANNEL_3 = 3,
  };

  void set_bayer_pattern(BAYER_PATTERN v)
  {
    _bayer_pattern = v;
  }

  BAYER_PATTERN bayer_pattern() const
  {
    return _bayer_pattern;
  }

  void set_display_channel(DISPLAY_CHANNEL v)
  {
    _display_channel = v;
  }

  DISPLAY_CHANNEL display_channel() const
  {
    return _display_channel;
  }

  void set_differentiate(bool v)
  {
    _differentiate = v;
  }

  bool differentiate() const
  {
    return _differentiate;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;


protected:
  BAYER_PATTERN _bayer_pattern =
      BAYER_RGGB;

  DISPLAY_CHANNEL _display_channel =
      DISPLAY_CHANNEL_0;

  bool _differentiate =
      false;
};

#endif /* __c_asi_frame_check_routine_h__ */
