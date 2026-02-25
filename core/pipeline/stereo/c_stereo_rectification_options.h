/*
 * c_stereo_rectification_options.h
 *
 *  Created on: Jul 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_rectification_options_h__
#define __c_stereo_rectification_options_h__

#include <core/proc/camera_calibration/stereo_calibrate.h>
#include <core/settings/opencv_settings.h>

class c_stereo_rectification_options
{
public:
  void set_enabled(bool v);
  bool enabled() const;

  void set_camera_intrinsics_yml(const std::string & v);
  const std::string& camera_intrinsics_yml() const;

  void set_camera_extrinsics_yml(const std::string & v);
  const std::string& camera_extrinsics_yml() const;

  void set_has_changes(bool v);
  bool has_changes() const;

  bool serialize(c_config_setting settings, bool save);

protected:
  std::string _camera_intrinsics_yml;
  std::string _camera_extrinsics_yml;
  bool _enabled = false;
  bool _has_changes = false;
};

#endif /* __c_stereo_rectification_options_h__ */
