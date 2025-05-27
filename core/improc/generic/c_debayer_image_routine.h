/*
 * c_debayer_image_routine.h
 *
 *  Created on: May 26, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_debayer_image_routine_h__
#define __c_debayer_image_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/io/debayer.h>

class c_debayer_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_debayer_image_routine,
      "debayer", "debayer raw monochrome image");

  void set_colorid(enum COLORID v)
  {
    _colorid = v;
  }

  enum COLORID colorid() const
  {
    return _colorid;
  }

  void set_method(enum DEBAYER_ALGORITHM v)
  {
    _method = v;
  }

  enum DEBAYER_ALGORITHM method() const
  {
    return _method;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  enum COLORID _colorid = COLORID_BAYER_RGGB;
  enum DEBAYER_ALGORITHM _method = DEBAYER_NN2;
};

#endif /* __c_debayer_image_routine_h__ */
