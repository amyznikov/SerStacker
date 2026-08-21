/*
 * c_canvas_average_pipeline.h
 *
 *  Created on: Feb 24, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_canvas_average_pipeline_h__
#define __c_canvas_average_pipeline_h__

#include <core/pipeline/c_image_stacking_pipeline_base/c_image_stacking_pipeline_base.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>
#include <core/proc/sharpness_measure/c_local_variance_sharpness_measure.h>
#include <core/improc/c_image_processor.h>
#include <core/average/c_frame_accumulation.h>
#include <core/settings/opencv_settings.h>

struct c_canvas_average_input_options:
    c_image_stacking_pipeline_base_input_options
{
 //     c_image_processor::sptr ecc_image_processor;
};

struct c_canvas_average_registration_options
{
  bool enable_star_registration = false;
  bool enable_ecc_registration = false;
  bool enable_eccflow_registration = false;

  cv::Size canvasSize;
  double eccUnsharpMaskSigma = 1;
  double eccUnsharpMaskAlpha = 0.9;


  IMAGE_MOTION_TYPE motion_type = IMAGE_MOTION_TRANSLATION;

  c_simple_star_detector_options star_detection;
  c_triangle_extractor_options triangle_extractor;
  c_triangle_matcher_options triangle_matcher;

  c_ecch_options ecch;
  c_eccflow_options eccflow;
  c_estimate_image_transform_options transform_estimation;
};

struct c_canvas_average_update_options
{
  //double running_weight = 15000;
  c_canvas_average::options update;
  c_local_variance_map_options sharpness_measure;
};

struct c_canvas_average_output_options:
    c_image_processing_pipeline_output_options
{
  double display_scale = 0;

  int autoSaveInterval = 1000;

  bool save_progress_video = false;
  bool save_input_video = false;
  bool save_reference_video = false;
  bool save_weights_video = false;
  c_output_frame_writer_options output_input_video_options;
  c_output_frame_writer_options output_progress_video_options;
  c_output_frame_writer_options output_reference_video_options;
  c_output_frame_writer_options output_weights_video_options;
  std::string output_file_name;

};

class c_canvas_average_pipeline :
  public c_image_stacking_pipeline_base
{
public:
  typedef c_canvas_average_pipeline this_class;
  typedef c_image_stacking_pipeline_base base;
  typedef std::shared_ptr<this_class> sptr;

  c_canvas_average_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string& get_class_name() const override;
  static const std::string& class_name();
  static const std::string& tooltip();

  const c_enum_member * get_preview_displays() const override;
  bool serialize(c_config_setting settings, bool save) override;
  bool copy_parameters(const c_image_processing_pipeline::sptr & dst) const override;
  static const c_ctlist<this_class> & getcontrols();


protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  bool process_current_frame();
  void compute_weights(const cv::Mat & src, const cv::Mat & srcmask,  cv::Mat & dst);
  std::string generate_output_file_name() const;
  bool write_input_video(cv::InputArray image, cv::InputArray mask);
  bool write_progress_video(cv::InputArray image, cv::InputArray mask);
  bool write_reference_video(cv::InputArray image, cv::InputArray mask);
  bool write_weights_video(cv::InputArray image, cv::InputArray mask);
  bool save_averaged_image();

protected:
  c_canvas_average_input_options _input_options;
  c_canvas_average_registration_options _registration_options;
  c_canvas_average_update_options _average_options;
  c_canvas_average_output_options _output_options;
  int _input_bpp = 0;

  c_ecch _ecch;
  c_eccflow _eccflow;
  c_star_extractor _star_extractor;
  c_triangle_extractor _triangle_extractor;
  c_triangle_matcher _triangle_matcher;
  c_image_transform::sptr _image_transform;
  c_canvas_average _average;

  cv::Mat _current_image, _current_grayscale_image;
  cv::Mat _current_mask;
  mutable cv::Mat1f _apodizationWindow;

  c_output_frame_writer _input_video_writer;
  c_output_frame_writer _progress_writer;
  c_output_frame_writer _reference_video_writer;
  c_output_frame_writer _weights_video_writer;

  std::mutex _current_stars_lock;
  std::vector<cv::KeyPoint> _current_keypoints, _reference_keypoints;
  cv::Mat _current_descriptors, _reference_descriptors;
  std::vector<cv::DMatch> _current_matches;
};

#endif /* __c_canvas_average_pipeline_h__ */
