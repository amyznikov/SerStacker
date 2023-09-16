/*
 * c_generic_image_processor.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_generic_image_processor_h__
#define __c_generic_image_processor_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/improc/c_image_processor.h>
#include <core/settings/opencv_settings.h>
#include <core/io/c_output_frame_writer.h>
#include <core/proc/focus.h>

struct c_generic_image_processor_input_options :
    c_image_processing_pipeline_input_options
{
};

struct c_generic_image_processor_options
{
  c_image_processor::sptr image_processor;
//
//  bool adjust_sharpness = false;
//  double unsharp_sigma = 1;
};

struct c_generic_image_processor_output_options :
    c_image_processing_pipeline_output_options
{
  bool save_processed_frames;
  std::string processed_frames_filename;
};

class c_generic_image_processor_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_generic_image_processor_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_generic_image_processor_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "generic";
    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_generic_image_processor_pipeline.</strong><br>"
        "This pipeline uses specified c_image_processor for generic image processing<br>";
    return tooltip_;
  }

  const c_generic_image_processor_input_options & input_options() const;
  c_generic_image_processor_input_options & input_options();

  const c_generic_image_processor_options & processing_options() const;
  c_generic_image_processor_options & processing_options() ;

  const c_generic_image_processor_output_options & output_options() const;
  c_generic_image_processor_output_options & output_options();

  void set_output_file_name(const std::string & v);
  const std::string & output_file_name() const;

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame();
  //bool compute_averaged_sharpeness(int start_pos, int end_pos);

protected:
  c_generic_image_processor_input_options input_options_;
  c_generic_image_processor_options processing_options_;
  c_generic_image_processor_output_options output_options_;
  std::string output_file_name_;

  cv::Mat current_image_;
  cv::Mat current_mask_;

//  c_sharpness_norm_measure::ptr sharpness_norm_measure_;
//  double averaged_sharpeness_ = 0;

  c_output_frame_writer output_writer_;
};

#endif /* __c_generic_image_processor_h__ */
