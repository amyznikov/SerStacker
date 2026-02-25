/*
 * c_equalize_hist_routine.h
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_equalize_hist_routine_h__
#define __c_equalize_hist_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_equalize_hist_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_equalize_hist_routine,
      "equalize_hist", "Apply cv::equalizeHist() for each color channel");

  bool process(cv::InputOutputArray _image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
};

#endif /* __c_equalize_hist_routine_h__ */
