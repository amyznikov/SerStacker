/*
 * c_image_undistorsion_routine.h
 *
 *  Created on: May 31, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_undistorsion_routine_h__
#define __c_image_undistorsion_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/camera_calibration/camera_calibration.h>

class c_image_undistorsion_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_image_undistorsion_routine,
      "undistorsion",
      "c_image_undistorsion_routine");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::vector<double> _dist_coeffs;
};

#endif /* __c_image_undistorsion_routine_h__ */
