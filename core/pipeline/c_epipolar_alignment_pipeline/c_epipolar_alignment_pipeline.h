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
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/settings/opencv_settings.h>
#include <core/settings/camera_settings.h>

struct c_epipolar_alignment_input_options:
    c_image_processing_pipeline_input_options
{
};

struct c_epipolar_alignment_camera_options
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

enum c_epipolar_alignment_feature2d_type
{
  c_epipolar_alignment_feature2d_sparse,
  c_epipolar_alignment_feature2d_dense,
};

struct c_epipolar_alignment_feature2d_options
{
  c_epipolar_alignment_feature2d_type feature2d_type = c_epipolar_alignment_feature2d_sparse;
  c_sparse_feature_detector_options sparse_detector_options;
  c_optflowpyrlk_feature2d_matcher_options optflowpyrlk_options;
};

struct c_epipolar_alignment_output_options:
    c_image_processing_pipeline_output_options
{
    bool save_progress_video = false;
    bool save_optflow_video = false;
    bool save_matches_csv = false;
    c_output_frame_writer_options progress_output_options;
    c_output_frame_writer_options optflow_output_options;
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
  //static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();
  bool copyParameters(const base::sptr & dst) const override;
  static const c_ctlist<this_class> & getcontrols();

protected:
  bool initialize() override;
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame();
  bool extract_and_match_keypoints();
  bool extract_and_match_keypoints_sparse();
  bool extract_and_match_keypoints_dense();
  bool estmate_camera_pose();
  bool fuse_matches();
  bool save_progess_videos();
  bool save_matches_csv();

protected:
  c_epipolar_alignment_input_options _input_options;
  c_epipolar_alignment_camera_options _camera_options;
  c_epipolar_alignment_feature2d_options _feature2d_options;
  c_lm_camera_pose_options _camera_pose_options;
  c_epipolar_alignment_output_options _output_options;

  cv::Mat1f G;
  double correlation_threshold = 0.7;
  double correlation_eps = 1e-4;
  int correlation_kernel_radius = 5;

  c_eccflow _eccflow;
  c_feature2d::sptr _sparse_feature_detector;

  std::vector<cv::KeyPoint> _current_keypoints;
  std::vector<cv::KeyPoint> _previous_keypoints;
  std::vector<cv::Point2f> _matched_current_positions;
  std::vector<cv::Point2f> _matched_previous_positions;
  std::vector<cv::Point2f> _warped_current_positions;

  std::vector<cv::Point2f> _matched_fused_positions;

  cv::Mat _current_frame, _current_mask;
  cv::Mat _previous_frame, _previous_mask;
  cv::Mat current_mg, previous_mg, remapped_current_mg;
  cv::Mat2f _current_remap;
  cv::Mat1f _current_correlation;
  cv::Mat1b _current_correlation_mask;
  cv::Mat1b _current_inliers;

  cv::Vec3d _current_euler_anges;
  cv::Vec3d _current_translation_vector;
  cv::Matx33d _currentRotationMatrix;
  cv::Matx33d _currentDerotationHomography;
  cv::Point2d _currentEpipole;

  c_output_frame_writer _progress_writer;
  c_output_frame_writer _optflow_writer;
};

#endif /* __c_epipolar_alignment_pipeline_h__ */
