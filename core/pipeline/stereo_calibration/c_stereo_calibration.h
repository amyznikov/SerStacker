/*
 * c_stereo_calibration.h
 *
 *  Created on: Mar 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_calibration_h__
#define __c_stereo_calibration_h__

#include <core/proc/chessboard/chessboard_detection.h>
#include <core/proc/camera_calibration/stereo_calibrate.h>

enum STEREO_CALIBRATION_STAGE {
  stereo_calibration_idle = 0,
  stereo_calibration_initialize,
  stereo_calibration_in_progress,
  stereo_calibration_finishing
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

  double filter_alpha = 0.5;
};


struct c_stereo_calibration_output_options
{
  std::string output_directory;
  std::string rectified_frames_filename;
  std::string stereo_rectified_frames_filename;
  std::string quad_rectified_frames_filename;
  std::string calibration_progress_filename;

  bool save_rectified_frames = false;
  bool save_stereo_rectified_frames = false;
  bool save_quad_rectified_frames = false;
  bool save_calibration_progress_video = false;
};

class c_stereo_calibration
{
public:
  typedef c_stereo_calibration this_class;

  c_stereo_calibration() = default;
  virtual ~c_stereo_calibration() = default;

  c_chessboard_corners_detection_options & chessboard_detection_options();
  const c_chessboard_corners_detection_options & chessboard_detection_options() const;

  c_stereo_calibrate_options & stereo_calibrate_options();
  const c_stereo_calibrate_options & stereo_calibrate_options() const;

  c_stereo_calibration_output_options & output_options();
  const c_stereo_calibration_output_options & output_options() const;

  void set_output_intrinsics_filename(const std::string & v);
  const std::string& output_intrinsics_filename() const;

  void set_output_extrinsics_filename(const std::string & v);
  const std::string & output_extrinsics_filename() const;

  bool serialize(c_config_setting setting, bool save);

  virtual bool initialize();
  virtual void cleanup();
  virtual bool process_stereo_frame(const cv::Mat images[2], cv::Mat masks[2]);
  virtual void update_display_image();
  virtual bool get_display_image(cv::Mat *display_frame, cv::Mat * display_mask);

protected:
  virtual bool canceled();
  virtual bool detect_chessboard(const cv::Mat &frame, std::vector<cv::Point2f> & corners_) const;
  virtual double estimate_grid_subset_quality(int excludedIndex) const;
  virtual void filter_frames();
  virtual void update_state();
  virtual void update_undistortion_remap();
  virtual bool save_current_camera_parameters() const;

protected:
  double estimate_subset_quality() const;
  void estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const;

protected:
  c_chessboard_corners_detection_options chessboard_detection_options_;
  c_stereo_calibrate_options calibration_options_;
  c_stereo_calibration_output_options output_options_;

  STEREO_CALIBRATION_STAGE pipeline_stage_ = stereo_calibration_idle;

  cv::Mat current_frames_[2];
  cv::Mat current_masks_[2];

  std::vector<cv::Point2f> current_image_points_[2];
  std::vector<cv::Point3f> current_object_points_;

  std::vector<std::vector<cv::Point2f> > image_points_[2];
  std::vector<std::vector<cv::Point3f> > object_points_;

  c_stereo_camera_intrinsics stereo_intrinsics_;
  c_stereo_camera_extrinsics stereo_extrinsics_;
  int calibration_flags_ = 0;
  bool stereo_intrinsics_initialized_ = false;

  c_stereo_camera_intrinsics best_intrinsics_;
  c_stereo_camera_extrinsics best_extrinsics_;
  int best_calibration_flags_ = 0;
  double best_subset_quality_ = HUGE_VAL;


  c_stereo_camera_intrinsics new_stereo_intrinsics_;
  c_stereo_camera_extrinsics new_stereo_extrinsics;
  cv::Mat2f rmaps_[2];
  cv::Matx33d E_;
  cv::Matx33d F_;
  std::vector<cv::Vec3d> rvecs_;
  std::vector<cv::Vec3d> tvecs_;
  cv::Mat1d perViewErrors_;
  double rmse_ = 0;

  cv::Mat display_frame_;
  cv::Mat display_mask_;

  std::string output_intrinsics_filename_;
  std::string output_extrinsics_filename_;

};

#endif /* __c_stereo_calibration_h__ */
