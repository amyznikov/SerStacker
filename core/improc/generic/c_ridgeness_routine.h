/*
 * c_ridgeness_routine.h
 *
 *  Created on: May 1, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_ridgeness_routine_h__
#define __c_ridgeness_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/cdiffs.h>

class c_ridgeness_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_ridgeness_routine,
      "ridgeness", "Compute hessian-based ridgeness on image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
};

#endif /* __c_ridgeness_routine_h__ */
