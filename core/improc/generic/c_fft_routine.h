/*
 * c_fft_routine.h
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_routine_h__
#define __c_fft_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/fft.h>

class c_fft_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_routine,
      "fft", "Display fft spectrum from cv::dft()");

  enum DisplayType {
    DisplayPower,
    DisplayPhase,
    DisplayReal,
    DisplayImag,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DisplayType _output_display = DisplayPower;
  bool _dft_scale = false;
  //cv::BorderTypes _borderType = cv::BORDER_REFLECT101;

};

#endif /* __c_fft_routine_h__ */
