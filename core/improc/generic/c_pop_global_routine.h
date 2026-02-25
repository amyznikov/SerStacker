/*
 * c_pop_global_routine.h
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pop_global_routine_h__
#define __c_pop_global_routine_h__

#include <core/improc/c_image_processor.h>

class c_pop_global_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_pop_global_routine,
      "pop_global", "c_pop_global_routine");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::string _artifact_name  = "saved_image";
};

#endif /* __c_pop_global_routine_h__ */
