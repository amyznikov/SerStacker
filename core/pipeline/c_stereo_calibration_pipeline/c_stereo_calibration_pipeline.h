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

struct c_stereo_calibration_input_options :
    c_stereo_input_options
{
};



struct c_stereo_calibrate_options
{
  int min_frames = 10;
  int max_frames = 50;

  int calibration_flags = STEREO_CALIB_USE_INTRINSIC_GUESS | STEREO_CALIB_FIX_ASPECT_RATIO |
      STEREO_CALIB_SAME_FOCAL_LENGTH | STEREO_CALIB_ZERO_TANGENT_DIST |
      STEREO_CALIB_FIX_K4 | STEREO_CALIB_FIX_K5 | STEREO_CALIB_FIX_K6 | STEREO_CALIB_FIX_S1_S2_S3_S4 |
      STEREO_CALIB_FIX_TAUX_TAUY;

  bool enable_calibration = true;
  bool auto_tune_calibration_flags = true;
  bool init_camera_matrix_2d = true;

  int max_iterations = 50;
  double solver_eps = 1e-6;
  double filter_alpha = 0.5;
};

struct c_stereo_calibration_output_options :
    c_image_processing_pipeline_output_options
{
  std::string output_intrinsics_filename;
  std::string output_extrinsics_filename;

  bool save_chessboard_frames = false;
  bool save_rectified_frames = false;
  bool save_stereo_rectified_frames = false;
  bool save_quad_output_frames = false;
  bool save_quad_rectified_frames = false;
  bool save_progress_video = false;

  c_output_frame_writer_options chessboard_frames_output_options;
  c_output_frame_writer_options rectified_frames_output_options;
  c_output_frame_writer_options stereo_rectified_output_options;
  c_output_frame_writer_options quad_output_options;
  c_output_frame_writer_options quad_rectified_output_options;
  c_output_frame_writer_options progress_video_output_options;
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
  bool copyParameters(const base::sptr & dst) const override;
  static const c_ctlist<this_class> & getcontrols();

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;

protected:
  bool open_input_source();
  void close_input_source();
  bool seek_input_source(int pos);
  bool read_stereo_frame();
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
  c_stereo_input_source _input;
  c_stereo_calibration_input_options _input_options;
  c_chessboard_corners_detection_options _chessboard_detection_options;
  c_stereo_calibrate_options _calibration_options;
  c_stereo_calibration_output_options _output_options;

  cv::Mat _current_frames[2];
  cv::Mat _current_masks[2];

  std::vector<cv::Point2f> _current_image_points[2];
  std::vector<cv::Point3f> _current_object_points;

  std::vector<std::vector<cv::Point2f> > _image_points[2];
  std::vector<std::vector<cv::Point3f> > _object_points;

  c_stereo_camera_intrinsics _current_intrinsics;
  c_stereo_camera_extrinsics _current_extrinsics;
  int _current_calibration_flags = 0;
  bool _stereo_intrinsics_initialized = false;

  c_stereo_camera_intrinsics _best_intrinsics;
  c_stereo_camera_extrinsics _best_extrinsics;
  int _best_calibration_flags = 0;
  double _best_subset_quality = DBL_MAX;

  c_stereo_camera_intrinsics _new_intrinsics;
  c_stereo_camera_extrinsics _new_extrinsics;
  cv::Mat2f _rmaps[2];
  cv::Matx33d _E;
  cv::Matx33d _F;
  cv::Mat1d _perViewErrors;
  double _rmse = 0;

  std::string _output_intrinsics_filename;
  std::string _output_extrinsics_filename;
  c_output_frame_writer _chessboard_video_writer;
  bool _writing_output_videos = false;
};

#endif /* __c_stereo_calibration_pipeline_h__ */
