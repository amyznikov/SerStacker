/*
 * c_anscombe_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_anscombe_routine_h__
#define __c_anscombe_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_anscombe_transform.h>

class c_anscombe_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_anscombe_routine,
      "anscombe", "Apply anscombe transform to image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_anscombe_transform anscombe;
  bool invert = false;
};



#endif /* __c_anscombe_routine_h__ */
