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

#include "c_image_processor.h"
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
    intrinsics_filename_  = v;
    camera_intrinsics_initialized_ = false;
  }

  const std::string & intrinsics_filename() const
  {
    return intrinsics_filename_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, intrinsics_filename, "Camera intrinsics YML file");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {

      c_config_setting section;

      SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);

      return true;
    }

    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    if ( !camera_intrinsics_initialized_ ) {

      if( intrinsics_filename_.empty() ) {
        return false;
      }

      if( !read_camera_intrinsics_yml(&intrinsics_, intrinsics_filename_) ) {
        CF_ERROR("read_camera_intrinsics_yml('%s') fails",
            intrinsics_filename_.c_str());
        return false;
      }

      cv::initUndistortRectifyMap(intrinsics_.camera_matrix,
          intrinsics_.dist_coeffs,
          cv::noArray(),
          cv::getOptimalNewCameraMatrix(intrinsics_.camera_matrix, intrinsics_.dist_coeffs,
              intrinsics_.image_size, 0.0, intrinsics_.image_size),
          intrinsics_.image_size,
          CV_32FC2,
          rmap,
          cv::noArray());

      camera_intrinsics_initialized_ = true;
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

protected:
  std::string intrinsics_filename_;
  c_camera_intrinsics intrinsics_;
  bool camera_intrinsics_initialized_ = false;
  cv::Mat2f rmap;
};

#endif /* __c_image_rectification_routine_h__ */
