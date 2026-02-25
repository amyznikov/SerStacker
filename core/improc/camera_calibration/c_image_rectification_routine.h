/*
 * c_image_rectification_routine.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 *
 *  Apply camera image rectification remap to input image.
 *
 *
 *  The rectification remap is constructed using cv::initUndistortRectifyMap()
 *  from data read from user-provided calibration YML file
 *  created by OpenCV or camera calibration pipeline.
 *  For actual YML file format see c_camra_calibration::save_current_camera_parameters()
 *
 */

#pragma once
#ifndef __c_image_rectification_routine_h__
#define __c_image_rectification_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/camera_calibration/camera_calibration.h>

class c_image_rectification_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_image_rectification_routine,
      "image_rectification",
      "Apply camera rectification initrinsics to input image.");

  void set_intrinsics_filename(const std::string & v)
  {
    _intrinsics_filename  = v;
    _camera_intrinsics_initialized = false;
  }

  const std::string & intrinsics_filename() const
  {
    return _intrinsics_filename;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::string _intrinsics_filename;
  c_camera_intrinsics _intrinsics;
  bool _camera_intrinsics_initialized = false;
  cv::Mat2f rmap;
};

#endif /* __c_image_rectification_routine_h__ */
