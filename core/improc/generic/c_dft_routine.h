/*
 * c_dft_routine.h
 *
 *  Created on: Nov 1, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dft_routine_h__
#define __c_dft_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/fft.h>

class c_dft_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_dft_routine,
      "dft", "App.y cv::dft() r cv::idft() to single-channel input image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  bool _dftScale = false;
  bool _dftComplexOutput = false;
  bool _polar = true;
  bool _centerDC = true;
};

#endif /* __c_dft_routine_h__ */
