/*
 * c_camera_calibration_pipeline.h
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 *
 * Based on /opencv/apps/interactive-calibration
 */

#pragma once
#ifndef __c_chessboard_camera_calibration_pipeline_h__
#define __c_chessboard_camera_calibration_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/proc/chessboard/chessboard_detection.h>
#include <core/proc/camera_calibration/calibrate_camera.h>


struct c_camera_calibration_input_options :
    c_image_processing_pipeline_input_options
{
};

struct c_calibrate_camera_options
{
  int min_frames = 10;
  int max_frames = 50;
  int calibration_flags = CAMERA_CALIB_USE_INTRINSIC_GUESS;
  bool enable_calibration = true;
  bool auto_tune_calibration_flags = true;
  bool init_camera_matrix_2d = true;

  int max_iterations = 30;
  double solver_eps = 1e-6;

  double filter_alpha = 0.5;
};

struct c_camera_calibration_output_options :
    c_image_processing_pipeline_output_options
{
  bool save_chessboard_frames = false;
  bool save_rectified_frames = false;
  bool save_coverage_frame = true;
  bool save_debug_video = false;

  std::string output_intrinsics_filename;
  std::string output_coverage_frame_filename;

  c_output_frame_writer_options output_chessboard_video_options;
  c_output_frame_writer_options output_rectified_video_options;
  c_output_frame_writer_options output_debug_video_options;
};


class c_camera_calibration_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_camera_calibration_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_camera_calibration_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_camera_calibration_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "camera_calibration";
    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_camera_calibration_pipeline.</strong><br>"
        "This pipeline uses cv::calibrateCamera() for camera calibration<br>";
    return tooltip_;
  }

  c_camera_calibration_input_options & input_options();
  const c_camera_calibration_input_options & input_options() const;

  c_chessboard_corners_detection_options & chessboard_corners_detection_options();
  const c_chessboard_corners_detection_options & chessboard_corners_detection_options() const;

  c_calibrate_camera_options & calibrate_camera_options();
  const c_calibrate_camera_options & calibrate_camera_options() const;

  c_camera_calibration_output_options & output_options();
  const c_camera_calibration_output_options & output_options() const;

  bool serialize(c_config_setting setting, bool save) override;
  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();
  bool copyParameters(const base::sptr & dst) const override;

protected:
  bool initialize_pipeline()override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;

protected:
  //bool run_chessboard_corners_collection();
  bool open_input_sequence();
  void close_input_sequence();
  bool seek_input_sequence(int pos);
  bool read_input_frame(const c_input_sequence::sptr & input_sequence, cv::Mat & output_image, cv::Mat & output_mask);
  bool process_current_frame(bool enable_calibration);
  bool detect_chessboard(const cv::Mat &frame);
  void filter_frames(bool only_landmarks);
  double estimate_subset_quality() const;
  double estimate_coverage_quality(int excludedIndex) const;
  void estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const;
  bool update_calibration();
  void update_state();
  void update_undistortion_remap();
  bool save_current_camera_parameters() const;
  bool write_chessboard_video();
  bool write_output_videos();

protected:
  c_camera_calibration_input_options input_options_;
  c_chessboard_corners_detection_options chessboard_detection_options_;
  c_calibrate_camera_options calibration_options_;
  c_camera_calibration_output_options output_options_;
  cv::Mat missing_pixel_mask_;

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

  c_output_frame_writer chessboard_video_writer_;
};

#endif /* __c_chessboard_camera_calibration_pipeline_h__ */
