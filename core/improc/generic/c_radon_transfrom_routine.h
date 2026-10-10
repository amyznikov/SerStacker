/*
 * c_radon_transfrom_routine.h
 *
 *  Created on: Oct 10, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_radon_transfrom_routine_h__
#define __c_radon_transfrom_routine_h__

#include <core/improc/c_image_processor.h>

class c_radon_transfrom_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_radon_transfrom_routine,
      "radon_transfrom", "DFT-based RadonTransform");

  enum DISPLAY {
    DISPLAY_SINOGRAM,
    DISPLAY_P_SPECTRUM,
    DISPLAY_CMLPX_SPEC_CART,
    DISPLAY_CMLPX_SPEC_CART_POLAR,
    DISPLAY_CMLPX_SPEC_POLAR,
    DISPLAY_CMLPX_SPEC_POLAR_POLAR
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_SINOGRAM;
  double theta = 1;
  double start_angle = 0;
  double end_angle = 180;
  bool crop = false;
  bool norm = false;
};

#endif /* __c_radon_transfrom_routine_h__ */
