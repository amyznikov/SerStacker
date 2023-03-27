/*
 * c_regular_stereo_pipeline.h
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_regular_stereo_pipeline_h__
#define __c_regular_stereo_pipeline_h__

#include "c_image_processing_pipeline.h"
#include "rstereo/c_regular_stereo.h"

#include <core/feature2d/feature2d.h>
#include <core/proc/camera_calibration/camera_calibration.h>
#include <core/improc/c_image_processor.h>
#include <core/io/c_output_frame_writer.h>
#include <core/proc/stereo/c_scale_sweep_stereo_matcher.h>

enum RSTEREO_CALIBRATION_STAGE {
  rstereo_calibration_idle = 0,
  rstereo_calibration_initialize,
  rstereo_calibration_in_progress,
  rstereo_calibration_finishing
};

struct c_regular_stereo_input_options
{
  std::string left_stereo_source;
  std::string right_stereo_source;

  int start_frame_index = 0;
  int max_input_frames = -1;

  bool convert_to_grayscale = false;
  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;

  // Extracted from P_rect_00 of KITTI calib_cam_to_cam.txt
  // for .image_size = cv::Size(1242, 375),
  cv::Matx23d cameraMatrix = cv::Matx23d(
      7.215377e+02, 0.000000e+00, 6.095593e+02,
      0.000000e+00, 7.215377e+02, 1.728540e+02);
};


struct c_regular_stereo_feature2d_options {
  double scale = 0.5;
  c_sparse_feature_extractor_options sparse_feature_extractor;
  c_feature2d_matcher_options sparse_feature_matcher;
};


struct c_regular_stereo_calibratie_options
{
  bool enable_calibration = true;
  bool apply_stereo_rectification = true;
  std::string calibration_config_filename; // input

  int min_frames = 3;
  int max_frames = 50;
  double filter_alpha = 0.3;

};

struct c_regular_stereo_matching_options
{
  bool enable_stereo_matchning = true;

  int max_disparity = 128;
  int max_scale = 2;
  int kernel_radius = 3;
  double kernel_sigma = 1;

  bool save_debug_images = false;
  bool process_only_debug_frames = false;
  std::vector<int> debug_frames;
  std::vector<cv::Point> debug_points;

};

struct c_regular_stereo_image_processing_options
{
  c_image_processor::sptr input_image_processor;
  c_image_processor::sptr stereo_match_preprocessor;
  c_image_processor::sptr output_image_processor;
};


struct c_regular_stereo_output_options
{
  bool save_calibration_config_file = true;
  std::string calibration_config_filename; // output

  bool save_progress_video = false;
  std::string progress_video_filename;

  bool save_rectified_videos = false;
  std::string rectified_video_filenames[2];

  std::string & left_rectified_video_filename =
      rectified_video_filenames[0];

  std::string & right_rectified_video_filename =
      rectified_video_filenames[0];

  bool save_stereo_matches_video = false;
  std::string stereo_matches_video_filename;

  bool save_motion_poses = false;
  std::string motion_poses_filename;

  bool save_stereo_match_progress_video = false;
  std::string stereo_match_progress_video_filename;

};

class c_regular_stereo_pipeline:
    public c_image_processing_pipeline
{
public:
  typedef c_regular_stereo_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_regular_stereo_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_regular_stereo_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "rstereo";

    return classname_;
  }

  c_regular_stereo_input_options & input_options();
  const c_regular_stereo_input_options & input_options() const;

  c_regular_stereo_feature2d_options & feature2d_options();
  const c_regular_stereo_feature2d_options & feature2d_options() const;

  c_regular_stereo_calibratie_options & calibration_options();
  const c_regular_stereo_calibratie_options & calibration_options() const;

  c_regular_stereo_matching_options & stereo_matching_options();
  const c_regular_stereo_matching_options & stereo_matching_options() const;

  c_regular_stereo_image_processing_options & image_processing_options() ;
  const c_regular_stereo_image_processing_options & image_processing_options() const;

  c_regular_stereo_output_options & output_options();
  const c_regular_stereo_output_options & output_options() const;

  bool serialize(c_config_setting setting, bool save) override;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

  c_notification<void()> on_current_frame_changed;
  c_notification<void()> on_accumulator_changed;

protected:
  //void update_output_path() override;
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool run_calibration();
  bool run_stereo_matching();

  void reset_input_frames();
  bool open_input_streams();
  bool read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const;
  bool read_stereo_frame();
  bool detect_keypoints();

  void update_calibration_display_image(bool applyHomography = false, bool drawmatches = false, int stream_pos = -1);
  bool write_calibration_progress_video(c_output_frame_writer & w);

  void update_disparity_map_display_image(const cv::Mat & currentFrame, const cv::Mat & referenceFrame,
      const cv::Mat1w & disparityMap,
      const cv::Mat1b & disparityMask,
      int max_disparity);

  std::string get_calibraton_config_output_filename() const;
  bool load_calibration_config_file();
  bool save_calibration_config_file() const;
  bool save_rectified_videos();

protected:

  /**
   * Struct with data from a stereo pair
   */
  struct c_stereo_frame {
    // Inputs from stereo camera, filled by get_next_stero_frame()
    cv::Mat images[2]; // left and right images
    cv::Mat masks[2]; // left and right image masks
    double pts = 0; // stereo frame time stamp

    // Keypoints and matches()
    std::vector<cv::KeyPoint> keypoints[2]; // current (left) and reference (right) keypoints
    cv::Mat descriptors[2]; // current (left) and reference (right) descriptors
  };

  struct c_motion_pose {
    cv::Point2d E[2]; // [left/right]
    cv::Vec3d T[2]; // [left/right]
    double rmse[2];   // [left/right]
    std::vector<cv::Point2f> matched_positions[2]; // [left/right]
    int stream_pos;
  };

  bool compute_motion_pose(int camera_index, c_motion_pose * pose) const;
  bool detect_current_stereo_matches(c_motion_pose * pose);

  std::vector<c_motion_pose> motion_poses_;


  cv::Mat missing_pixel_mask_;
  c_regular_stereo_input_options input_options_;
  c_regular_stereo_feature2d_options feature2d_options_;
  c_regular_stereo_calibratie_options calibration_options_;
  c_regular_stereo_matching_options stereo_matching_options_;
  c_regular_stereo_image_processing_options image_processing_options_;
  c_regular_stereo_output_options output_options_;

  c_camera_intrinsics intrinsics_;
  //cv::Matx33d cameraMatrix;

  mutable std::mutex accumulator_lock_;

  c_input_source::sptr input_sources_[2];
  c_stereo_frame input_frames_[2];

  c_sparse_feature_extractor::ptr keypoints_extractor_;
  c_feature2d_matcher::ptr keypoints_matcher_;

  c_stereo_frame * current_frame_ = nullptr;
  c_stereo_frame * previous_frame_ = nullptr;

  cv::Matx33d rectificationHomography[2];
  bool haveRectificationHomography = false;

  c_stereo_camera_intrinsics stereo_intrinsics_;
  c_stereo_camera_extrinsics stereo_extrinsics_;
  //bool stereo_intrinsics_initialized_ = false;

  c_stereo_camera_intrinsics new_stereo_intrinsics_;
  c_stereo_camera_extrinsics new_stereo_extrinsics;
  cv::Mat2f rmap_;
  cv::Matx33d E_;
  cv::Matx33d F_;

  mutable std::mutex display_lock_;
  cv::Mat display_frame_;
  cv::Mat display_mask_;

  double rmse_ = 0;
  int calibration_flags_ = 0;

  FILE * posesfp_ = nullptr;
  void dump_motion_pose(const c_motion_pose & pose);
};

#endif /* __c_regular_stereo_pipeline_h__ */
