/*
 * c_load_image_routine.h
 *
 *  Created on: May 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_load_image_routine_h__
#define __c_load_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_load_image_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_load_image_routine,
      "load_image", "Load image from file");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::string _filename;
  std::string _artifact_name  = "saved_image";
};

#endif /* __c_load_image_routine_h__ */
