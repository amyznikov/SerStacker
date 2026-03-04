/*
 * c_guided_filter_routine.h
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_guided_filter_routine_h__
#define __c_guided_filter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_guided_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_guided_filter_routine,
      "c_guided_filter_routine", "Apply cv::ximgproc::guidedFilter()");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double eps = 1.0;
  double scale = 1.0;
  int radius = 3;
  int mkgr = 0;
  PIXEL_DEPTH ddepth = PIXEL_DEPTH_NO_CHANGE;
};

#endif /* __c_guided_filter_routine_h__ */
