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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  enum COLORID _colorid = COLORID_BAYER_RGGB;
  enum DEBAYER_ALGORITHM _method = DEBAYER_NN2;
};

#endif /* __c_debayer_image_routine_h__ */
