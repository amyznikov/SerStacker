/*
 * c_pnormalize_routine.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 *
 *  Subtract local mean and divide by local stdev
 */

#pragma once
#ifndef __c_pnormalize_routine_h__
#define __c_pnormalize_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>
#include <core/proc/pyrscale.h>

class c_pnormalize_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_pnormalize_routine, "pnormalize",
      "Subtract local mean and divide by stdev");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale, PIXEL_DEPTH ddepth);
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _scale = 3;
  PIXEL_DEPTH _ddepth = PIXEL_DEPTH_32F;

};

#endif /* __c_pnormalize_routine_h__ */
