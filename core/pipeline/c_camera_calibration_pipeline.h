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

#include "c_image_processing_pipeline.h"
#include <core/proc/chessboard/chessboard_detection.h>
#include <core/proc/camera_calibration/calibrate_camera.h>

enum CAMERA_CALIBRATION_STAGE {
  camera_calibration_idle = 0,
  camera_calibration_initialize,
  camera_calibration_in_progress,
  camera_calibration_finishing
};

struct c_camera_calibration_input_options
{
  int start_frame_index = 0;
  int max_input_frames = -1;

  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;
};

struct c_calibrate_camera_options
{
  int min_frames = 3;
  int max_frames = 50;
  int calibration_flags = CAMERA_CALIB_USE_INTRINSIC_GUESS;
  bool auto_tune_calibration_flags = true;

  cv::TermCriteria solverTerm =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
          50, 1e-7);

  double filter_alpha = 0.1;
};

struct c_camera_calibration_output_options
{
  bool save_rectified_images = false;
  std::string rectified_images_file_name;
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

  void set_chessboard_size(const cv::Size & v);
  const cv::Size& chessboard_size() const;

  void set_chessboard_cell_size(const cv::Size2f & v);
  const cv::Size2f & chessboard_cell_size() const;

  c_camera_calibration_input_options & input_options();
  const c_camera_calibration_input_options & input_options() const;

  c_chessboard_corners_detection_options & chessboard_corners_detection_options();
  const c_chessboard_corners_detection_options & chessboard_corners_detection_options() const;

  c_calibrate_camera_options & calibrate_camera_options();
  const c_calibrate_camera_options & calibrate_camera_options() const;

  c_camera_calibration_output_options & output_options();
  const c_camera_calibration_output_options & output_options() const;

  bool serialize(c_config_setting setting, bool save) override;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

  c_notification<void()> on_current_frame_changed;
  c_notification<void()> on_accumulator_changed;
  c_notification<void(CAMERA_CALIBRATION_STAGE oldstage, CAMERA_CALIBRATION_STAGE newstage)> on_pipeline_stage_changed;

protected:
  bool initialize_pipeline()override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  void update_output_path() override;
  void set_pipeline_stage(CAMERA_CALIBRATION_STAGE stage);
  bool read_input_frame(const c_input_sequence::sptr & input_sequence, cv::Mat & output_image, cv::Mat & output_mask) const;
  bool detect_chessboard(const cv::Mat &frame);
  void update_undistortion_remap();
  void update_display_image();
  double estimate_grid_subset_quality(size_t excludedIndex) const;
  double estimate_coverage_quality() const;
  void filter_frames();
  void update_state();
  bool save_current_camera_parameters() const;

protected:
  cv::Size chessboard_size_ = cv::Size(9, 6);
  cv::Size2f chessboard_cell_size_ = cv::Size2f(0.09, 0.09);

  cv::Mat missing_pixel_mask_;
  c_camera_calibration_input_options input_options_;
  c_chessboard_corners_detection_options chessboard_corners_detection_options_;
  c_calibrate_camera_options calibration_options_;
  c_camera_calibration_output_options output_options_;

  CAMERA_CALIBRATION_STAGE pipeline_stage_ = camera_calibration_idle;

  mutable std::mutex accumulator_lock_;
  cv::Mat current_frame_;
  cv::Mat current_mask_;
  cv::Mat display_frame_;
  cv::Mat display_mask_;

  std::vector<cv::Point2f> current_image_points_;
  std::vector<cv::Point3f> current_object_points_;

  c_camera_intrinsics intrinsics_;
  cv::Mat1d stdDeviations_;
  cv::Mat1d perViewErrors_;
  cv::Mat2f current_undistortion_remap_;
  double rmse_ = 0;

  bool intrinsics_initialized_ = false;


  std::vector<std::vector<cv::Point2f> > image_points_;
  std::vector<std::vector<cv::Point3f> > object_points_;

  bool is_chessboard_found_ = false;
  int calibration_flags_ = 0;
  bool confIntervalsState_ = false;
  bool coverageQualityState_ = false;


};

#endif /* __c_chessboard_camera_calibration_pipeline_h__ */