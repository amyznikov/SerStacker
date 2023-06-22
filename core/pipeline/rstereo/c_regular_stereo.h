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
#include <core/improc/c_image_processor.h>
#include <core/settings/opencv_settings.h>

struct c_regular_stereo_image_processing_options
{
  c_image_processor::sptr input_image_processor;
  c_image_processor::sptr remapped_image_processor;
  c_image_processor::sptr output_image_processor;
};

struct c_stereo_output_options
{
  std::string output_directory;
  std::string progress_video_filename;
  std::string depthmap_filename;
  std::string cloud3d_image_filename;
  std::string cloud3d_ply_filename;

  bool save_progress_video = false;
  bool save_depthmaps = true;
  bool save_cloud3d_image = true;
  bool save_cloud3d_ply = true;
};

class c_regular_stereo
{
public:
  c_regular_stereo() = default;
  virtual ~c_regular_stereo() = default;

  void set_enable_stereo_rectification(bool v);
  bool enable_stereo_rectification() const;

  void set_camera_intrinsics_yml(const std::string & v);
  const std::string& camera_intrinsics_yml() const;

  void set_camera_extrinsics_yml(const std::string & v);
  const std::string& camera_extrinsics_yml() const;

  c_regular_stereo_matcher& stereo_matcher();
  const c_regular_stereo_matcher& stereo_matcher() const;

  c_regular_stereo_image_processing_options & image_processing_options() ;
  const c_regular_stereo_image_processing_options & image_processing_options() const;

  c_stereo_output_options & output_options();
  const c_stereo_output_options & output_options() const;

protected:
  bool initialize();
  void cleanup();
  bool process_stereo_frame(const cv::Mat images[2], const cv::Mat masks[2]);

  virtual bool canceled() const;
  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask);


protected:
  std::string camera_intrinsics_yml_;
  std::string camera_extrinsics_yml_;

  c_regular_stereo_matcher stereo_matcher_;
  c_regular_stereo_image_processing_options image_processing_options_;
  c_stereo_output_options output_options_;

  bool enable_stereo_rectification_ = true;
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
  //cv::Mat current_display_;

};

#endif /* __c_regular_stereo_h__ */
