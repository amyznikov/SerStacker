/*
 * c_stereo_rectification_options.cc
 *
 *  Created on: Jul 9, 2023
 *      Author: amyznikov
 */

#include "c_stereo_rectification_options.h"

void c_stereo_rectification_options::set_enabled(bool v)
{
  _enabled = v;
}

bool c_stereo_rectification_options::enabled() const
{
  return _enabled;
}

void c_stereo_rectification_options::set_has_changes(bool v)
{
  _has_changes = v;
}

bool c_stereo_rectification_options::has_changes() const
{
  return _has_changes;
}

void c_stereo_rectification_options::set_camera_intrinsics_yml(const std::string & v)
{
  _camera_intrinsics_yml = v;
  _has_changes = true;
}

const std::string& c_stereo_rectification_options::camera_intrinsics_yml() const
{
  return _camera_intrinsics_yml;
}

void c_stereo_rectification_options::set_camera_extrinsics_yml(const std::string & v)
{
  _camera_extrinsics_yml = v;
  _has_changes = true;
}

const std::string& c_stereo_rectification_options::camera_extrinsics_yml() const
{
  return _camera_extrinsics_yml;
}

//const c_stereo_camera_intrinsics & c_stereo_rectification_options::stereo_intrinsics() const
//{
//  return stereo_intrinsics_;
//}
//
//const c_stereo_camera_extrinsics & c_stereo_rectification_options::stereo_extrinsics() const
//{
//  return stereo_extrinsics_;
//}

bool c_stereo_rectification_options::serialize(c_config_setting settings, bool save)
{
  SERIALIZE_PROPERTY(settings, save, *this, enabled);
  SERIALIZE_PROPERTY(settings, save, *this, camera_intrinsics_yml);
  SERIALIZE_PROPERTY(settings, save, *this, camera_extrinsics_yml);
  return true;
}

