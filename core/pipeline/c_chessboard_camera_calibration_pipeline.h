/*
 * c_chessboard_camera_calibration_pipeline.h
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
#include <core/proc/chessboard/chessboard_camera_calibration.h>

enum CHESSBOARD_CAMERA_CALIBRATION_STAGE {
  chessboard_camera_calibration_idle = 0,
  chessboard_camera_calibration_initialize,
  chessboard_camera_calibration_detect_chessboard_corners,
  chessboard_camera_calibration_in_progress,
  chessboard_camera_calibration_finishing
};

enum CAMERA_CALIBRATION_FLAGS
{
  CAMERA_CALIBRATION_USE_INTRINSIC_GUESS = cv::CALIB_USE_INTRINSIC_GUESS,
  CAMERA_CALIBRATION_FIX_ASPECT_RATIO = cv::CALIB_FIX_ASPECT_RATIO,
  CAMERA_CALIBRATION_FIX_PRINCIPAL_POINT = cv::CALIB_FIX_PRINCIPAL_POINT,
  CAMERA_CALIBRATION_ZERO_TANGENT_DIST = cv::CALIB_ZERO_TANGENT_DIST,
  CAMERA_CALIBRATION_FIX_FOCAL_LENGTH = cv::CALIB_FIX_FOCAL_LENGTH,
  CAMERA_CALIBRATION_FIX_K1 = cv::CALIB_FIX_K1,
  CAMERA_CALIBRATION_FIX_K2 = cv::CALIB_FIX_K2,
  CAMERA_CALIBRATION_FIX_K3 = cv::CALIB_FIX_K3,
  CAMERA_CALIBRATION_FIX_K4 = cv::CALIB_FIX_K4,
  CAMERA_CALIBRATION_FIX_K5 = cv::CALIB_FIX_K5,
  CAMERA_CALIBRATION_FIX_K6 = cv::CALIB_FIX_K6,
  CAMERA_CALIBRATION_RATIONAL_MODEL = cv::CALIB_RATIONAL_MODEL,
  CAMERA_CALIBRATION_THIN_PRISM_MODEL = cv::CALIB_THIN_PRISM_MODEL,
  CAMERA_CALIBRATION_FIX_S1_S2_S3_S4 = cv::CALIB_FIX_S1_S2_S3_S4,
  CAMERA_CALIBRATION_TILTED_MODEL = cv::CALIB_TILTED_MODEL,
  CAMERA_CALIBRATION_FIX_TAUX_TAUY = cv::CALIB_FIX_TAUX_TAUY,
  CAMERA_CALIBRATION_USE_QR = cv::CALIB_USE_QR, //!< use QR instead of SVD decomposition for solving. Faster but potentially less precise
  CAMERA_CALIBRATION_FIX_TANGENT_DIST = cv::CALIB_FIX_TANGENT_DIST,
  CAMERA_CALIBRATION_USE_LU = cv::CALIB_USE_LU, //!< use LU instead of SVD decomposition for solving. much faster but potentially less precise
};

struct c_chessboard_camera_calibration_input_options
{
  int start_frame_index = 0;
  int max_input_frames = -1;

  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;
};

struct c_chessboard_camera_calibration_options
{
  int min_frames = 3;
  int max_frames = 50;
  int calibration_flags = 0; // enum CAMERA_CALIBRATION_FLAGS
  bool auto_tune_calibration_flags = true;

  cv::TermCriteria solverTerm =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
          50, 1e-7);

  double filter_alpha = 0.1;
};

struct c_chessboard_camera_calibration_output_options
{
  bool save_rectified_images = false;
  std::string rectified_images_file_name;
};


class c_chessboard_camera_calibration_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_chessboard_camera_calibration_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_chessboard_camera_calibration_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_chessboard_camera_calibration_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "chessboard_calibration";
    return classname_;
  }

  void set_chessboard_size(const cv::Size & v);
  const cv::Size& chessboard_size() const;

  void set_chessboard_cell_size(const cv::Size2f & v);
  const cv::Size2f & chessboard_cell_size() const;

  c_chessboard_camera_calibration_input_options & input_options();
  const c_chessboard_camera_calibration_input_options & input_options() const;

  c_chessboard_corners_detection_options & chessboard_corners_detection_options();
  const c_chessboard_corners_detection_options & chessboard_corners_detection_options() const;

  c_chessboard_camera_calibration_options & calibration_options();
  const c_chessboard_camera_calibration_options & calibration_options() const;

  c_chessboard_camera_calibration_output_options & output_options();
  const c_chessboard_camera_calibration_output_options & output_options() const;

  bool serialize(c_config_setting setting, bool save) override;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

  c_notification<void()> on_current_frame_changed;
  c_notification<void()> on_accumulator_changed;
  c_notification<void(CHESSBOARD_CAMERA_CALIBRATION_STAGE oldstage, CHESSBOARD_CAMERA_CALIBRATION_STAGE newstage)> on_pipeline_stage_changed;

protected:
  bool initialize_pipeline()override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  void update_output_path() override;
  void set_pipeline_stage(CHESSBOARD_CAMERA_CALIBRATION_STAGE stage);
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
  c_chessboard_camera_calibration_input_options input_options_;
  c_chessboard_corners_detection_options chessboard_corners_detection_options_;
  c_chessboard_camera_calibration_options calibration_options_;
  c_chessboard_camera_calibration_output_options output_options_;

  CHESSBOARD_CAMERA_CALIBRATION_STAGE pipeline_stage_ = chessboard_camera_calibration_idle;

  mutable std::mutex accumulator_lock_;
  cv::Mat current_frame_;
  cv::Mat current_mask_;
  cv::Mat display_frame_;
  cv::Mat display_mask_;

  std::vector<cv::Point2f> current_image_points_;
  std::vector<cv::Point3f> current_object_points_;
  cv::Mat current_camera_matrix_;
  cv::Mat current_dist_coeffs_;
  cv::Mat current_std_deviations_;
  cv::Mat current_per_view_errors_;
  cv::Mat2f current_undistortion_remap_;
  double current_total_avg_err_ = 0;

  std::vector<std::vector<cv::Point2f> > image_points_;
  std::vector< std::vector<cv::Point3f> > object_points_;

  bool isTemplateFound_ = false;
  int calibration_flags_ = 0;
  //unsigned mMinFramesNum;
  //bool mNeedTuning = false;
  bool mConfIntervalsState = false;
  bool mCoverageQualityState = false;


};

#endif /* __c_chessboard_camera_calibration_pipeline_h__ */
