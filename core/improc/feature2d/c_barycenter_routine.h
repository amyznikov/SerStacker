/*
 * c_barycenter_routine.h
 *
 *  Created on: May 30, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_barycenter_routine_h__
#define __c_barycenter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/ctrlbind/ctrlbind.h>

class c_barycenter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_barycenter_routine,
      "c_barycenter_routine", "Use pf cv::moment to compute image barycenter on ROI");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  bool _update_pointer = true;
};

#endif /* __c_barycenter_routine_h__ */
