/*
 * c_push_image_routine.h
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_push_current_image_routine_h__
#define __c_push_current_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_push_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_push_image_routine,
      "push_image", "c_push_image_routine");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::string _artifact_name  = "saved_image";
  bool _push_image = true;
  bool _push_mask = true;

};

#endif /* __c_push_current_image_routine_h__ */
