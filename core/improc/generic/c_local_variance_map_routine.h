/*
 * c_local_variance_map_routine.h
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_local_variance_map_routine_h__
#define __c_local_variance_map_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/sharpness_measure/c_local_variance_sharpness_measure.h>

class c_local_variance_map_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_local_variance_map_routine,
      "local_variance_map", "Use cv::boxFilter() for local variance map");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_local_variance_map_options _opts;
};

#endif /* __c_local_variance_map_routine_h__ */
