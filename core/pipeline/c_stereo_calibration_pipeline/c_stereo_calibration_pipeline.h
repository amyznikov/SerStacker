/*
 * c_stereo_calibration_pipeline.h
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_calibration_pipeline_h__
#define __c_stereo_calibration_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/pipeline/stereo/c_stereo_input_options.h>
#include <core/proc/chessboard/chessboard_detection.h>
#include <core/proc/camera_calibration/stereo_calibrate.h>
#include <core/io/c_output_frame_writer.h>

struct c_stereo_calibration_input_options :
    c_stereo_input_options
{
};

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


struct c_stereo_calibration_output_options :
    c_image_processing_pipeline_output_options
{
  std::string output_intrinsics_filename;
  std::string output_extrinsics_filename;
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

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_stereo_calibration_pipeline.</strong><br>"
        "This pipeline uses cv::stereoCalibrate() <br>"
        "for stereo camera calibration<br>";
    return tooltip_;
  }

  c_stereo_calibration_input_options & input_options();
  const c_stereo_calibration_input_options & input_options() const;

  c_chessboard_corners_detection_options & chessboard_detection_options();
  const c_chessboard_corners_detection_options & chessboard_detection_options() const;

  c_stereo_calibrate_options & calibration_options();
  const c_stereo_calibrate_options & calibration_options() const;

  c_stereo_calibration_output_options & output_options();
  const c_stereo_calibration_output_options & output_options() const;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;
  bool serialize(c_config_setting setting, bool save) override;

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;

protected:
  bool open_input_source();
  void close_input_source();
  bool seek_input_source(int pos);
  bool read_stereo_frame();
  //bool read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const;
  // bool run_chessboard_corners_collection();
  bool process_current_stereo_frame(bool enable_calibration);
  bool detect_chessboard(const cv::Mat & frame, std::vector<cv::Point2f> & corners_) const;
  bool update_calibration();
  void update_state();
  void update_undistortion_remap();
  void estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const;
  double estimate_coverage_quality(int excludedIndex) const;
  double estimate_subset_quality() const;
  void filter_frames(bool only_landmarks);
  bool save_current_camera_parameters() const;
  bool write_output_videos();
  bool write_chessboard_video();

protected:
  c_stereo_input_source input_;
  c_stereo_calibration_input_options input_options_;
  c_chessboard_corners_detection_options chessboard_detection_options_;
  c_stereo_calibrate_options calibration_options_;
  c_stereo_calibration_output_options output_options_;

  cv::Mat current_frames_[2];
  cv::Mat current_masks_[2];
  cv::Mat missing_pixel_mask_;

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
  std::string rectified_frames_filename_;
  std::string stereo_rectified_frames_filename_;
  std::string quad_rectified_frames_filename_;
  std::string progress_video_filename_;

  c_output_frame_writer chessboard_video_writer_;
};

#endif /* __c_stereo_calibration_pipeline_h__ */
