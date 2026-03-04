/*
 * c_anisotropic_diffusion_filter_routine.h
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_anisotropic_diffusion_filter_routine_h__
#define __c_anisotropic_diffusion_filter_routine_h__

#include <core/improc/c_image_processor.h>

class c_anisotropic_diffusion_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_anisotropic_diffusion_filter_routine,
      "anisotropic_diffusion", "Apply Perona-Malik anisotropic diffusion cv::ximgproc::anisotropicDiffusion(src, dst, alpha, K, niters);");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  float alpha = 0.1f;
  float K = 0.02f;
  int niters = 10;
};

#endif /* __c_anisotropic_diffusion_filter_routine_h__ */
