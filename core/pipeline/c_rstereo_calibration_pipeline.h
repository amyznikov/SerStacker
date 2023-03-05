/*
 * c_rstereo_calibration_pipeline.h
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rstereo_calibration_pipeline_h__
#define __c_rstereo_calibration_pipeline_h__

#include "c_image_processing_pipeline.h"
#include <core/proc/camera_calibration/camera_calibration.h>
#include <core/feature2d/feature2d.h>


enum RSTEREO_CALIBRATION_STAGE {
  rstereo_calibration_idle = 0,
  rstereo_calibration_initialize,
  rstereo_calibration_in_progress,
  rstereo_calibration_finishing
};

struct c_rstereo_calibration_input_options
{
  std::string left_stereo_source;
  std::string right_stereo_source;

  int start_frame_index = 0;
  int max_input_frames = -1;

  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;
};


struct c_rstereo_feature2d_options {
  double scale = 0.5;
  c_sparse_feature_extractor_options sparse_feature_extractor;
  c_feature2d_matcher_options sparse_feature_matcher;
};


struct c_rstereo_calibrate_options
{
  int min_frames = 3;
  int max_frames = 50;
  double filter_alpha = 0.3;
};

struct c_rstereo_calibration_output_options
{
  bool save_rectified_images = false;
  std::string rectified_images_file_name;
};

class c_rstereo_calibration_pipeline:
    public c_image_processing_pipeline
{
public:
  typedef c_rstereo_calibration_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_rstereo_calibration_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_rstereo_calibration_pipeline();

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

  c_rstereo_calibration_input_options & input_options();
  const c_rstereo_calibration_input_options & input_options() const;

  c_rstereo_feature2d_options & feature2d_options();
  const c_rstereo_feature2d_options & feature2d_options() const;

  c_rstereo_calibrate_options & stereo_calibrate_options();
  const c_rstereo_calibrate_options & stereo_calibrate_options() const;

  c_rstereo_calibration_output_options & output_options();
  const c_rstereo_calibration_output_options & output_options() const;

  bool serialize(c_config_setting setting, bool save) override;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

  c_notification<void()> on_current_frame_changed;
  c_notification<void()> on_accumulator_changed;

protected:
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  void update_output_path() override;
  bool open_input_streams();
  bool read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const;
  bool read_stereo_frame();
  bool detect_and_match_keypoints();
  void update_remap();
  void update_display_image(bool drawpoints =  true);
  double estimate_grid_subset_quality(int excludedIndex) const;
  void filter_frames();
  void update_state();
  bool save_current_camera_parameters() const;
  std::string generate_video_file_name(const std::string & ufilename, const std::string & postfix) const;

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
    std::vector<cv::DMatch> matches; // matches between left and right frame sparse features (keypoints)
    //std::vector<uint8_t> matched_inliers; // mask for matched inliers
    std::vector<cv::Point2f> matched_positions[2];
  };


  cv::Mat missing_pixel_mask_;
  c_rstereo_calibration_input_options input_options_;
  c_rstereo_feature2d_options feature2d_options_;
  c_rstereo_calibrate_options calibration_options_;
  c_rstereo_calibration_output_options output_options_;

  mutable std::mutex accumulator_lock_;

  c_input_source::sptr input_sources_[2];

  c_sparse_feature_extractor::ptr keypoints_extractor_;
  c_feature2d_matcher::ptr keypoints_matcher_;

  c_stereo_frame current_frame_;

  std::vector<std::vector<cv::Point2f>> matched_positions[2];
  std::vector<double> perViewErrors_;



  c_stereo_camera_intrinsics stereo_intrinsics_;
  c_stereo_camera_extrinsics stereo_extrinsics_;
  bool stereo_intrinsics_initialized_ = false;

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
};

#endif /* __c_rstereo_calibration_pipeline_h__ */
