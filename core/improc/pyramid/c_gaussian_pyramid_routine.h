/*
 * c_gaussian_pyramid_routine.h
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pyrdown_routine_h__
#define __c_pyrdown_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/unsharp_mask.h>

class c_gaussian_pyramid_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gaussian_pyramid_routine, "gaussian_pyramid",
      "calls cv::pyrDown() or cv::pyrUp() on image");

  enum SharpenOrder {
    SharpenNone,
    SharpenBefore,
    SharpenAfter,
    SharpenEachIteration,
  };


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _count = 1;
  cv::BorderTypes _borderType = cv::BORDER_DEFAULT;
  SharpenOrder _sharpen_order = SharpenNone;
  double _sharpen_amount = 0.7;
  double _sharpen_outmin = 0;
  double _sharpen_outmax = 1e6;
};

#endif /* __c_pyrdown_routine_h__ */
