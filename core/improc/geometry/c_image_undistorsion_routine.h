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


  void set_dist_coeffs(const std::vector<double> & v)
  {
    dist_coeffs_ = v;
  }

  const std::vector<double> & dist_coeffs() const
  {
    return dist_coeffs_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override;

protected:
  std::vector<double> dist_coeffs_;

};

#endif /* __c_image_undistorsion_routine_h__ */
