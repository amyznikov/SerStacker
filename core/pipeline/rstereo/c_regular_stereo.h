/*
 * c_regular_stereo.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_regular_stereo_h__
#define __c_regular_stereo_h__

#include <core/proc/stereo/c_regular_stereo_matcher.h>
#include <core/proc/camera_calibration/camera_calibration.h>
#include <core/settings/opencv_settings.h>

class c_regular_stereo
{
public:
  c_regular_stereo() = default;
  virtual ~c_regular_stereo() = default;

  void set_camera_intrinsics_yml(const std::string & v);
  const std::string& camera_intrinsics_yml() const;

  void set_camera_extrinsics_yml(const std::string & v);
  const std::string& camera_extrinsics_yml() const;

  c_regular_stereo_matcher& stereo_matcher();
  const c_regular_stereo_matcher& stereo_matcher() const;

  bool serialize(c_config_setting settings, bool save);

  virtual bool initialize();
  virtual void cleanup();
  virtual bool process_stereo_frame(const cv::Mat images[2], const cv::Mat masks[2]);
  virtual void update_display_image();
  virtual bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask);

protected:
  virtual bool canceled();

protected:
  std::string camera_intrinsics_yml_;
  std::string camera_extrinsics_yml_;
  c_regular_stereo_matcher stereo_matcher_;
  c_stereo_camera_intrinsics stereo_intrinsics_;
  c_stereo_camera_extrinsics stereo_extrinsics_;
  c_stereo_camera_intrinsics new_intrinsics_;
  c_stereo_camera_extrinsics new_extrinsics_;
  cv::Mat2f rmaps_[2];
  cv::Matx33d R_[2];
  cv::Matx34d P_[2];
  cv::Matx44d Q_;
  cv::Rect validRoi_[2];

  cv::Mat current_images_[2];
  cv::Mat current_masks_[2];
  cv::Mat current_disparity_;
  cv::Mat current_display_;

};

#endif /* __c_regular_stereo_h__ */
