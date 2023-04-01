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
#include <core/io/c_output_frame_writer.h>

struct c_stereo_calibrate_options
{
  int min_frames = 10;
  int max_frames = 50;
  int calibration_flags = STEREO_CALIB_USE_INTRINSIC_GUESS | STEREO_CALIB_FIX_ASPECT_RATIO;
  bool enable_calibration = true;
  bool auto_tune_calibration_flags = true;
  bool init_camera_matrix_2d = false;

  cv::TermCriteria solverTerm =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
          50, 1e-7);

  double filter_alpha = 0.5;
};


struct c_stereo_calibration_output_options
{
  std::string output_directory;
  std::string chessboard_frames_filename;
  std::string rectified_frames_filename;
  std::string stereo_rectified_frames_filename;
  std::string quad_rectified_frames_filename;
  std::string progress_video_filename;

  bool save_chessboard_frames = false;
  bool save_rectified_frames = false;
  bool save_stereo_rectified_frames = false;
  bool save_quad_rectified_frames = false;
  bool save_progress_video = false;
};

class c_stereo_calibration
{
public:
  typedef c_stereo_calibration this_class;

  c_stereo_calibration() = default;
  virtual ~c_stereo_calibration();

  c_chessboard_corners_detection_options & chessboard_detection_options();
  const c_chessboard_corners_detection_options & chessboard_detection_options() const;

  c_stereo_calibrate_options & calibration_options();
  const c_stereo_calibrate_options & calibration_options() const;

  c_stereo_calibration_output_options & output_options();
  const c_stereo_calibration_output_options & output_options() const;

protected:
  void set_output_intrinsics_filename(const std::string & v);
  const std::string& output_intrinsics_filename() const;

  void set_output_extrinsics_filename(const std::string & v);
  const std::string & output_extrinsics_filename() const;

  void set_chessboard_frames_filename(const std::string & v);
  const std::string & chessboard_frames_filename() const;


  bool initialize();
  void cleanup();
  bool process_stereo_frame(const cv::Mat images[2], const cv::Mat masks[2], bool enable_calibration);
  bool process_current_stereo_frame(bool enable_calibration);

  virtual bool canceled() const;
  virtual bool serialize(c_config_setting setting, bool save);
  virtual bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask);

  bool detect_chessboard(const cv::Mat &frame, std::vector<cv::Point2f> & corners_) const;
  void update_state();
  void update_undistortion_remap();
  bool save_current_camera_parameters() const;
  void estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const;
  double estimate_coverage_quality(int excludedIndex) const;
  double estimate_subset_quality() const;
  void filter_frames(bool only_landmarks);
  bool update_calibration();
  bool write_horizontal_stereo_video();

protected:
  c_chessboard_corners_detection_options chessboard_detection_options_;
  c_stereo_calibrate_options calibration_options_;
  c_stereo_calibration_output_options output_options_;

  cv::Mat current_frames_[2];
  cv::Mat current_masks_[2];

  std::vector<cv::Point2f> current_image_points_[2];
  std::vector<cv::Point3f> current_object_points_;

  std::vector<std::vector<cv::Point2f> > image_points_[2];
  std::vector<std::vector<cv::Point3f> > object_points_;

  c_stereo_camera_intrinsics current_intrinsics_;
  c_stereo_camera_extrinsics current_extrinsics_;
  int current_calibration_flags_ = 0;
  bool stereo_intrinsics_initialized_ = false;

  c_stereo_camera_intrinsics best_intrinsics_;
  c_stereo_camera_extrinsics best_extrinsics_;
  int best_calibration_flags_ = 0;
  double best_subset_quality_ = HUGE_VAL;


  c_stereo_camera_intrinsics new_intrinsics_;
  c_stereo_camera_extrinsics new_extrinsics_;
  cv::Mat2f rmaps_[2];
  cv::Matx33d E_;
  cv::Matx33d F_;
//  std::vector<cv::Vec3d> rvecs_;
//  std::vector<cv::Vec3d> tvecs_;
  cv::Mat1d perViewErrors_;
  double rmse_ = 0;

  std::string output_intrinsics_filename_;
  std::string output_extrinsics_filename_;
  std::string chessboard_frames_filename_;

  c_output_frame_writer chessboard_video_writer_;

  //  cv::Mat display_frame_;
  //  cv::Mat display_mask_;

};

#endif /* __c_stereo_calibration_h__ */
