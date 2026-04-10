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
#include <core/roi_selection/c_roi_selection.h>
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


struct c_jdr_pipeline_master_options
{
  c_master_frame_selection_options master_selection;
};

struct c_jdr_pipeline_reference_frame_options
{
  std::string reference_file_name;
  bool generate_reference_frame = true;
  color_channel_type reference_channel = color_channel_gray;
//  c_image_processor::sptr input_image_preprocessor;

  //  bool stop_after_master_frame_generation = false;
  //  bool save_master_frame = true;
  //  double unsharp_sigma = 0;
  //  double unsharp_alpha = 0.8;
};


struct c_jdr_pipeline_stack_options {

};

struct c_jdr_pipeline_output_options  :
    c_image_processing_pipeline_output_options
{
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

  bool create_reeference_frame();

protected:
  c_jdr_pipeline_input_options _input_options;
  c_roi_selection_options _roi_selection_options;
  c_jdr_pipeline_master_options _master_options;
  c_jdr_pipeline_reference_frame_options  _reference_frame_options;
  c_jdr_pipeline_stack_options _stack_options;
  c_jdr_pipeline_output_options _output_options;

protected:
  c_roi_selection::sptr _roi_selection;
  int _pipeline_stage = 0;
};

#endif /* __c_jdr_pipeline_h__ */
