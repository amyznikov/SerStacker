/*
 * c_stereo_rectification_routine.h
 *
 *  Created on: Mar 23, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_rectification_routine_h__
#define __c_stereo_rectification_routine_h__

#include "c_image_processor.h"
#include <core/proc/camera_calibration/stereo_calibrate.h>

class c_stereo_rectification_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_stereo_rectification_routine,
      "stereo_rectification",
      "Apply stereo rectification to horizontal layout stereo frame");

  void set_intrinsics_filename(const std::string & v)
  {
    stereo_intrinsics_filename_  = v;
    stereo_calibration_initialized_ = false;
  }

  const std::string & intrinsics_filename() const
  {
    return stereo_intrinsics_filename_;
  }

  void set_extrinsics_filename(const std::string & v)
  {
    stereo_extrinsics_filename_  = v;
    stereo_calibration_initialized_ = false;
  }

  const std::string & extrinsics_filename() const
  {
    return stereo_extrinsics_filename_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, intrinsics_filename, "Stereo intrinsics yml file");
    ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_EXISTING_FILE(ctls, extrinsics_filename, "Stereo extrinsics yml file");

  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {

      c_config_setting section;

      SERIALIZE_PROPERTY(settings, save, *this, intrinsics_filename);
      SERIALIZE_PROPERTY(settings, save, *this, extrinsics_filename);

      return true;
    }

    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    if ( !stereo_calibration_initialized_ ) {

      if( !read_stereo_camera_intrinsics_yml(&intrinsics_, stereo_intrinsics_filename_) ) {
        CF_ERROR("read_stereo_camera_intrinsics_yml('%s') fails",
            stereo_intrinsics_filename_.c_str());
        return false;
      }

      if( !read_stereo_camera_extrinsics_yml(&extrinsics_, stereo_extrinsics_filename_) ) {
        CF_ERROR("read_stereo_camera_extrinsics_yml('%s') fails",
            stereo_extrinsics_filename_.c_str());
        return false;
      }

      CF_DEBUG("create_stereo_rectification()");

      bool fOk =
          create_stereo_rectification(cv::Size(image.cols() / 2, image.rows()),
              intrinsics_,
              extrinsics_,
              -1,
              rmaps);

      if ( !fOk ) {
        CF_ERROR("create_stereo_rectification() fails");
        return false;
      }

      CF_DEBUG("create_stereo_rectification() OK");

      stereo_calibration_initialized_ = true;
    }

    cv::Mat img =
        image.getMat();

    const cv::Rect roi[2] = {
        cv::Rect(0, 0, img.cols / 2, img.rows),
        cv::Rect(img.cols / 2, 0, img.cols / 2, img.rows),
    };

    for ( int i = 0; i < 2; ++i ) {
      cv::remap(img(roi[i]), img(roi[i]),
          rmaps[i], cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }

    return true;
  }

protected:

protected:
  std::string stereo_intrinsics_filename_;
  std::string stereo_extrinsics_filename_;

  c_stereo_camera_intrinsics intrinsics_;
  c_stereo_camera_extrinsics extrinsics_;

  bool stereo_calibration_initialized_ = false;
  cv::Mat2f rmaps[2];
};

#endif /* __c_stereo_rectification_routine_h__ */
