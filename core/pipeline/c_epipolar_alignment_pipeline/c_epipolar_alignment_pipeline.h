/*
 * c_epipolar_alignment_pipeline.h
 *
 *  Created on: Dec 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_epipolar_alignment_pipeline_h__
#define __c_epipolar_alignment_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/improc/c_image_processor.h>
#include <core/settings/opencv_settings.h>
#include <core/settings/camera_settings.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/feature2d/feature2d_settings.h>

struct c_epipolar_alignment_input_options:
    c_image_processing_pipeline_input_options
{
};

struct c_epipolar_alignment_stereo_camera_options
{
  c_camera_intrinsics camera_intrinsics = {
      // defaults to kitti
      .image_size = cv::Size(1242, 375),
      .camera_matrix =
          cv::Matx33d(
              7.215377e+02, 0.000000e+00, 6.095593e+02,
              0.000000e+00, 7.215377e+02, 1.728540e+02,
              0.000000e+00, 0.000000e+00, 1.000000e+00)
  };
};

struct c_epipolar_alignment_feature2d_options :
    public c_sparse_feature_extractor_and_matcher_options
{
};

struct c_epipolar_stereo_output_options:
    c_output_frame_writer_options
{
};

struct c_epipolar_stereo_scale_compression_output_options:
    c_output_frame_writer_options
{
};

struct c_epipolar_alignment_output_options:
    c_image_processing_pipeline_output_options
{
  bool save_epipolar_stereo_frames = false;
  bool save_scale_compression_remaps = false;
  c_epipolar_stereo_output_options epipolar_stereo_output_options;
  c_epipolar_stereo_scale_compression_output_options scale_compression_output_options;
};

class c_epipolar_alignment_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_epipolar_alignment_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_epipolar_alignment_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string class_name_ =
        "epipolar_alignment";
    return class_name_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_epipolar_alignment_pipeline.</strong><br>"
        "Epipolar alignment of pairs of consecutive video frames<br>";
    return tooltip_;
  }

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();
  bool copyParameters(const base::sptr & dst) const override;

protected:
  c_sparse_feature_extractor_and_matcher::sptr create_keypoints_extractor() const;
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame();
  bool extract_keypoints();
  bool match_keypoints();
  bool estmate_camera_pose();
  bool save_epipolar_stereo_frames();
  bool save_scale_compression_remaps();

protected:
  c_epipolar_alignment_input_options input_options_;
  c_epipolar_alignment_stereo_camera_options camera_options_;
  c_lm_camera_pose_options camera_pose_options_;
  c_epipolar_alignment_feature2d_options feature2d_options_;
  c_epipolar_alignment_output_options output_options_;

  cv::Mat current_frame_, current_mask_;
  cv::Mat previous_frame_, previous_mask_;

  c_sparse_feature_extractor_and_matcher::sptr keypoints_extractor_;

  std::vector<cv::KeyPoint> current_keypoints_;
  std::vector<cv::KeyPoint> previous_keypoints_;

  cv::Mat current_descriptors_;
  cv::Mat previous_descriptors_;

  cv::Mat1b current_inliers_;
  cv::Vec3d current_euler_anges_;
  cv::Vec3d current_translation_vector_;
  cv::Matx33d currentRotationMatrix_;
  cv::Matx33d currentEssentialMatrix_;
  cv::Matx33d currentFundamentalMatrix_;
  cv::Matx33d currentDerotationHomography_;
  cv::Point2d currentEpipoles_[2];
  cv::Point2d currentEpipole_;

  std::vector<cv::Point2f> matched_current_positions_;
  std::vector<cv::Point2f> matched_previous_positions_;



  c_output_frame_writer epipolar_stereo_writer_;
};

#endif /* __c_epipolar_alignment_pipeline_h__ */