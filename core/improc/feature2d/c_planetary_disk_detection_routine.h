/*
 * c_planetary_disk_detection_routine.h
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_planetary_disk_detection_routine_h__
#define __c_planetary_disk_detection_routine_h__

#include <core/improc/c_image_processor.h>

class c_planetary_disk_detection_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_planetary_disk_detection_routine,
      "planetary_disk_detection", "Calls simple_planetary_disk_detector()");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double gsigma = 1;
  int se_radius = 5;
  bool updateROI = false;
};

#endif /* __c_planetary_disk_detection_routine_h__ */
