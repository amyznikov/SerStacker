/*
 * c_cte_pipeline.h
 *
 *  Created on: Dec 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cte_pipeline_h__
#define __c_cte_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/proc/pose.h>
#include <core/proc/uvec3.h>
#include <deque>

struct c_cte_pipeline_input_options:
    c_image_processing_pipeline_input_options
{
   int read_step = -1;
};


struct c_cte_pipeline_camera_options
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

struct c_cte_pipeline_feature2d_options
{
  // Detector options
  c_sparse_feature_detector_options detector;

  // Descriptor options
  c_sparse_feature_descriptor_options descriptor;

  // Matcher options
  c_feature2d_matcher_options matcher;

  double image_scale = 1;
};

struct c_cte_context_options
{
  size_t max_context_size = 5;
};


struct c_cte_output_options:
    c_image_processing_pipeline_output_options
{
  bool save_progress_video = false;
  c_output_frame_writer_options progress_output_options;
};

struct c_cte_pose_estimation_options
{
  int max_iterations = 30;
  int max_levmar_iterations = 100;
  double levmar_epsf = 1e-15;
  double levmar_epsx = 1e-15;
  double robust_threshold = 15;
  double erfactor = 50;
};



class c_cte_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_cte_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_cte_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string & get_class_name() const final
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string class_name_ =
        "cte";
    return class_name_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_cte_pipeline.</strong><br>"
        "<br>";
    return tooltip_;
  }

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();
  bool copyParameters(const base::sptr & dst) const override;

protected:
  bool initialize() override;
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame();
  bool update_trajectory();
  bool save_progress_video();
  bool create_display_image(size_t back_frame_index, cv::OutputArray display_frame);

protected:
  c_cte_pipeline_input_options _input_options;
  c_cte_pipeline_camera_options _camera_options;
  c_cte_pipeline_feature2d_options _feature2d;
  c_cte_context_options _context_options;
  c_cte_pose_estimation_options _pose_estimation;
  c_cte_output_options _output_options;

protected:
  c_feature2d::sptr  _keypoints_detector;
  c_feature2d::sptr  _keypoints_descriptor;

protected:
  cv::Mat lastDisplayImage;
  c_output_frame_writer _progress_writer;

protected:

  struct c_cte_frame
  {
    typedef c_cte_frame this_class;
    typedef std::unique_ptr<this_class> uptr;

    cv::Mat image;
    cv::Mat mask;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    c_feature2d_matcher::sptr keypoints_matcher;
    std::vector<std::vector<int32_t>> matches;
    std::vector<cv::Mat1b> inliers;

    cv::Vec3d A =
        cv::Vec3d(0, 0, 0);

    UVec3d T =
        UVec3d (0, 0);

    cv::Matx33f H =
        cv::Matx33f::eye();
  };

  cv::Mat _current_image;
  cv::Mat _current_mask;

  std::deque<c_cte_frame::uptr> _frames;

};

#endif /* __c_cte_pipeline_h__ */
