/*
 * c_image_rectification_routine.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "c_image_rectification_routine.h"

void c_image_rectification_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_browse_for_file(ctls, "intrinsics", ctx,
      &this_class::intrinsics_filename, &this_class::set_intrinsics_filename,
      "Camera intrinsics YML file");
}

bool c_image_rectification_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);

    return true;
  }

  return false;
}

bool c_image_rectification_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !_camera_intrinsics_initialized ) {

    if( _intrinsics_filename.empty() ) {
      return false;
    }

    if( !read_camera_intrinsics_yml(&_intrinsics, _intrinsics_filename) ) {
      CF_ERROR("read_camera_intrinsics_yml('%s') fails",
          _intrinsics_filename.c_str());
      return false;
    }

    cv::initUndistortRectifyMap(_intrinsics.camera_matrix,
        _intrinsics.dist_coeffs,
        cv::noArray(),
        cv::getOptimalNewCameraMatrix(_intrinsics.camera_matrix, _intrinsics.dist_coeffs,
            _intrinsics.image_size, 0.0, _intrinsics.image_size),
        _intrinsics.image_size,
        CV_32FC2,
        rmap,
        cv::noArray());

    _camera_intrinsics_initialized = true;
  }

  cv::remap(image, image,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  if( mask.needed() ) {
    if( !mask.empty() ) {
      cv::remap(mask, mask,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }
    else {
      cv::remap(cv::Mat1b(image.size(), 255), mask,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }
    cv::compare(mask, 254, mask, cv::CMP_GE);
  }

  return true;
}

