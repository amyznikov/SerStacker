/*
 * c_fast_nl_means_denoising_routine.h
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fast_nl_means_denoising_routine_h__
#define __c_fast_nl_means_denoising_routine_h__

#include <core/improc/c_image_processor.h>


class c_fast_nl_means_denoising_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fast_nl_means_denoising_routine,
      "fast_nl_means_denoising", "Apply fastNlMeansDenoisingColored(src, dst,h, hColor,templateWindowSize, searchWindowSize)");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  float h = 3;
  float hColor = 3;
  int templateWindowRadius = 3;
  int searchWindowRadius = 10;
};

#endif /* __c_fast_nl_means_denoising_routine_h__ */
