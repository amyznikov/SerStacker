/*
 * c_camera_calibration.h
 *
 *  Created on: Mar 25, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_camera_calibration_h__
#define __c_camera_calibration_h__

#include <core/proc/chessboard/chessboard_detection.h>
#include <core/proc/camera_calibration/calibrate_camera.h>
#include <core/io/c_output_frame_writer.h>

struct c_calibrate_camera_options
{
  int min_frames = 10;
  int max_frames = 50;
  int calibration_flags = CAMERA_CALIB_USE_INTRINSIC_GUESS;
  bool enable_calibration = true;
  bool auto_tune_calibration_flags = true;
  bool init_camera_matrix_2d = true;

  cv::TermCriteria solverTerm =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
          50, 1e-7);

  double filter_alpha = 0.5;
};

struct c_camera_calibration_output_options
{
  std::string output_directory;
  std::string chessboard_frames_filename;
  std::string rectified_frames_filename;
  std::string progress_video_filename;

  bool save_chessboard_frames = false;
  bool save_rectified_frames = false;
  bool save_progress_video = false;
};

class c_camera_calibration
{
public:
  c_camera_calibration() = default;
  virtual ~c_camera_calibration();

  c_chessboard_corners_detection_options & chessboard_corners_detection_options();
  const c_chessboard_corners_detection_options & chessboard_corners_detection_options() const;

  c_calibrate_camera_options & calibrate_camera_options();
  const c_calibrate_camera_options & calibrate_camera_options() const;

  c_camera_calibration_output_options & output_options();
  const c_camera_calibration_output_options & output_options() const;

protected:

  void set_output_intrinsics_filename(const std::string & v);
  const std::string& output_intrinsics_filename() const;

  void set_chessboard_frames_filename(const std::string & v);
  const std::string & chessboard_frames_filename() const;

  virtual bool canceled() const;
  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

  bool initialize();
  void cleanup();
  bool process_frame(const cv::Mat & image, const cv::Mat & mask, bool enable_calibration);
  bool process_current_frame(bool enable_calibration);

  bool is_chessboard_found() const;

  bool detect_chessboard(const cv::Mat &frame);
  void update_state();
  void update_undistortion_remap();
  bool save_current_camera_parameters() const;
  void estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const; //
  double estimate_coverage_quality(int excludedIndex) const; //
  double estimate_subset_quality() const; //
  void filter_frames(bool only_landmarks);//
  bool update_calibration();
  bool write_chessboard_video();

protected:
  c_chessboard_corners_detection_options chessboard_detection_options_;
  c_calibrate_camera_options calibration_options_;
  c_camera_calibration_output_options output_options_;

  cv::Mat current_frame_;
  cv::Mat current_mask_;

  std::vector<cv::Point2f> current_image_points_;
  std::vector<cv::Point3f> current_object_points_;
  int current_calibration_flags_ = 0;

  c_camera_intrinsics current_intrinsics_;
  cv::Mat1d stdDeviations_;
  cv::Mat1d perViewErrors_;
  cv::Mat2f current_undistortion_remap_;
  double rmse_ = 0;

  c_camera_intrinsics best_intrinsics_;
  int best_calibration_flags_ = 0;
  double best_subset_quality_ = HUGE_VAL;


  std::vector<std::vector<cv::Point2f> > image_points_;
  std::vector<std::vector<cv::Point3f> > object_points_;

  bool intrinsics_initialized_ = false;
  bool is_chessboard_found_ = false;
  bool confIntervalsState_ = false;

  std::string output_intrinsics_filename_;
  std::string chessboard_frames_filename_;


  c_output_frame_writer chessboard_video_writer_;
};

#endif /* __c_camera_calibration_h__ */
