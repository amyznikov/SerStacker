/*
 * c_stereo_calibration_pipeline.h
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_calibration_pipeline_h__
#define __c_stereo_calibration_pipeline_h__

#include "c_image_processing_pipeline.h"
#include <core/proc/chessboard/chessboard_detection.h>
#include <core/proc/camera_calibration/stereo_calibrate.h>


enum STEREO_CALIBRATION_STAGE {
  stereo_calibration_idle = 0,
  stereo_calibration_initialize,
  stereo_calibration_in_progress,
  stereo_calibration_finishing
};

struct c_stereo_calibration_input_options
{
  std::string left_stereo_source;
  std::string right_stereo_source;

  int start_frame_index = 0;
  int max_input_frames = -1;

  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;
};

struct c_stereo_calibrate_options
{
  int min_frames = 3;
  int max_frames = 50;
  int calibration_flags = STEREO_CALIB_USE_INTRINSIC_GUESS;
  bool auto_tune_calibration_flags = true;

  cv::TermCriteria solverTerm =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
          50, 1e-7);

  double filter_alpha = 0.1;
};


struct c_stereo_calibration_output_options
{
  std::string rectified_frames_file_name;
  std::string stereo_rectified_frames_file_name;
  std::string quad_rectified_frames_file_name;

  bool save_rectified_frames = false;
  bool save_stereo_rectified_frames = false;
  bool save_quad_rectified_frames = false;
};

class c_stereo_calibration_pipeline:
    public c_image_processing_pipeline
{
public:
  typedef c_stereo_calibration_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_stereo_calibration_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_stereo_calibration_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "stereo_calibration";
    return classname_;
  }

  void set_chessboard_size(const cv::Size & v);
  const cv::Size& chessboard_size() const;

  void set_chessboard_cell_size(const cv::Size2f & v);
  const cv::Size2f & chessboard_cell_size() const;

  c_stereo_calibration_input_options & input_options();
  const c_stereo_calibration_input_options & input_options() const;

  c_chessboard_corners_detection_options & chessboard_corners_detection_options();
  const c_chessboard_corners_detection_options & chessboard_corners_detection_options() const;

  c_stereo_calibrate_options & stereo_calibrate_options();
  const c_stereo_calibrate_options & stereo_calibrate_options() const;

  c_stereo_calibration_output_options & output_options();
  const c_stereo_calibration_output_options & output_options() const;

  bool serialize(c_config_setting setting, bool save) override;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

  c_notification<void()> on_current_frame_changed;
  c_notification<void()> on_accumulator_changed;
  c_notification<void(STEREO_CALIBRATION_STAGE oldstage, STEREO_CALIBRATION_STAGE newstage)> on_pipeline_stage_changed;

protected:
  bool initialize_pipeline()override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool run_stereo_calibration();
  bool write_output_videos();
  void update_output_path() override;
  bool read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const;
  bool detect_chessboard(const cv::Mat &frame, std::vector<cv::Point2f> & corners_);
  void update_undistortion_remap();
  void update_display_image();
  double estimate_grid_subset_quality(int excludedIndex) const;
  void filter_frames();
  void update_state();
  bool save_current_camera_parameters() const;

protected:
  cv::Size chessboard_size_ = cv::Size(9, 6);
  cv::Size2f chessboard_cell_size_ = cv::Size2f(0.09, 0.09);

  cv::Mat missing_pixel_mask_;
  c_stereo_calibration_input_options input_options_;
  c_chessboard_corners_detection_options chessboard_corners_detection_options_;
  c_stereo_calibrate_options calibration_options_;
  c_stereo_calibration_output_options output_options_;

  STEREO_CALIBRATION_STAGE pipeline_stage_ = stereo_calibration_idle;

  mutable std::mutex accumulator_lock_;

  c_input_source::sptr input_sources_[2];

  cv::Mat current_frames_[2];
  cv::Mat current_masks_[2];

  std::vector<cv::Point2f> current_image_points_[2];
  std::vector<cv::Point3f> current_object_points_;

  std::vector<std::vector<cv::Point2f> > image_points_[2];
  std::vector<std::vector<cv::Point3f> > object_points_;

  c_stereo_camera_intrinsics stereo_intrinsics_;
  c_stereo_camera_extrinsics stereo_extrinsics_;
  bool stereo_intrinsics_initialized_ = false;


  c_stereo_camera_intrinsics new_stereo_intrinsics_;
  c_stereo_camera_extrinsics new_stereo_extrinsics;
  cv::Mat2f rmaps_[2];
  cv::Matx33d E_;
  cv::Matx33d F_;
  std::vector<cv::Vec3d> rvecs_;
  std::vector<cv::Vec3d> tvecs_;
  cv::Mat1d perViewErrors_;

  mutable std::mutex display_lock_;
  cv::Mat display_frame_;
  cv::Mat display_mask_;

  double rmse_ = 0;
  int calibration_flags_ = 0;
};

#endif /* __c_stereo_calibration_pipeline_h__ */
