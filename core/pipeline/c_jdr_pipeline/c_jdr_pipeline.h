/*
 * c_jdr_pipeline.h
 *
 *  Created on: Mar 17, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_jdr_pipeline_h__
#define __c_jdr_pipeline_h__

#include <core/pipeline/c_image_stacking_pipeline_base/c_image_stacking_pipeline_base.h>
#include <core/pipeline/c_master_frame_selection.h>
#include <core/proc/feature2d/c_roi_selection.h>
#include <core/proc/feature2d/c_jovian_ellipse_detector.h>
#include <core/proc/feature2d/c_jovian_derotation_remap.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/average/c_frame_accumulation.h>


struct c_jdr_pipeline_input_options :
    c_image_stacking_pipeline_base_input_options
{
};

struct c_jdr_pipeline_roi_options
{
  enum roi_selection_method method = roi_selection_none;
  cv::Size planetary_disk_crop_size;
  cv::Rect rectangle_roi_selection;
  double planetary_disk_gbsigma = 1;
  double planetary_disk_stdev_factor = 0.25;
  int se_close_size = 2;
};


struct c_jdr_pipeline_reference_frame_options
{
  std::string reference_file_name;
  c_master_frame_selection_options master_selection;
  bool generate_reference_frame = true;
  struct c_regerence_frame_generator_options {
    IMAGE_MOTION_TYPE motion_type = IMAGE_MOTION_TRANSLATION;
    color_channel_type reference_channel = color_channel_gray;
    c_image_processor::sptr input_image_preprocessor;
    c_ecch_options ecch_opts;
  } generate_opts;
};

struct c_jdr_pipeline_ellipsoid_pose {
  cv::Point2d center;
  cv::Vec3d axes;
  cv::Vec3d orientation;
};

struct c_jdr_pipeline_jovian_ellipse_detector_options
{
  c_jdr_pipeline_ellipsoid_pose pose;
  c_jovian_ellipse_detector_options jovian_ellipse_detector_options;
  bool auto_pose_estimation = true;
  bool update_estimated_pose = true;
};

struct c_jdr_pipeline_stack_options
{
  c_image_processor::sptr input_image_preprocessor;
  double wts = 120000; // [ms]
  int derotate_context_size = -1;
  bool derotate_all_frames = false;
};

struct c_jdr_pipeline_output_options: c_image_processing_pipeline_output_options
{
  bool save_aligned_frames = false;
  bool save_derotated_frames = false;
  bool save_accumulation_weights = false;
  c_output_frame_writer_options save_aligned_frames_opts;
  c_output_frame_writer_options save_derotated_frames_opts;
  c_output_frame_writer_options save_accumulation_weights_opts;
};


class c_jdr_pipeline :
    public c_image_stacking_pipeline_base
{
public:
  typedef c_jdr_pipeline this_class;
  typedef c_image_stacking_pipeline_base base;
  typedef std::shared_ptr<this_class> sptr;

  enum STACKING_STAGE
  {
    stacking_stage_idle = 0,
    stacking_stage_initialize,
    stacking_stage_select_master_frame_index,
    stacking_stage_generate_reference_frame,
    stacking_stage_in_progress,
    stacking_stage_finishing
  };

  c_jdr_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence = nullptr);
  ~c_jdr_pipeline();

  static const std::string & class_name();
  static const std::string & tooltip();
  const std::string & get_class_name() const final;

public:
  static const c_ctlist<this_class> & getcontrols();
  bool serialize(c_config_setting settings, bool save) override;
  bool copy_parameters(const c_image_processing_pipeline::sptr & dst) const override;

public:
  bool has_master_frame() const  final;
  void set_master_source(const std::string & master_source_path) final;
  std::string master_source() const final;
  void set_master_frame_index(int v) final;
  int master_frame_index() const final;

protected:
  bool initialize_pipeline() final;
  void cleanup_pipeline() final;
  bool run_pipeline() final;
  void set_pipeline_stage(int stage);
  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;

  bool create_reference_frame();
  bool estimate_jovian_ellipse();
  bool derotate_jovian_frames();
  bool open_output_writers();

  static bool preproc_align_and_remap(const c_image_processor::sptr & proc, c_ecch & ecch,
      cv::Mat & current_frame, cv::Mat & current_mask,
      color_channel_type reference_channel);

protected:
  c_jdr_pipeline_input_options _input_options;
  c_roi_selection_options _roi_selection_options;
  c_jdr_pipeline_reference_frame_options  _reference_frame_options;
  c_jdr_pipeline_jovian_ellipse_detector_options _ellipse_estimation_options;
  c_jdr_pipeline_stack_options _stack_options;
  c_jdr_pipeline_output_options _output_options;

protected:
  c_roi_selection::sptr _roi_selection;
  c_frame_weigthed_average _reference_frame_avg;
  c_jovian_ellipse_detector _jovian_ellipse_detector;
  c_jovian_derotation_remap _jovian_derotation_remap;
  c_frame_weigthed_average _frame_average;

  int _pipeline_stage = 0;

  cv::Mat _master_frame;
  cv::Mat _master_mask;
  cv::Mat _reference_frame;
  cv::Mat _reference_mask;
  cv::Mat _reference_planetary_disk_mask;
  cv::Mat _current_aligned_frame;
  cv::Mat _current_aligned_mask;
  double _master_ts = 0;
  int _master_pos = 0;
  c_jdr_pipeline_ellipsoid_pose _jovian_pose;

  c_output_frame_writer _current_aligned_frame_writer;
  c_output_frame_writer _current_derotated_frame_writer;
  c_output_frame_writer _accumulation_weights_writer;
};

#endif /* __c_jdr_pipeline_h__ */
