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
#include <core/proc/focus.h>

struct c_generic_image_processor_input_options :
    c_image_processing_pipeline_input_options
{
};

struct c_generic_image_processor_options
{
  c_image_processor::sptr image_processor;
};


struct c_generic_image_processor_output_options :
    c_image_processing_pipeline_output_options
{
  bool save_processed_frames;
  c_output_frame_writer_options processed_file_options;
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

  const std::string & get_class_name() const override;
  static const std::string & class_name();
  static const std::string & tooltip();

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  bool copy_parameters(const base::sptr & dst) const override;

  static const c_ctlist<this_class> & getcontrols();

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame();

protected:
  c_generic_image_processor_input_options _input_options;
  c_generic_image_processor_options _processing_options;
  c_generic_image_processor_output_options _output_options;

  cv::Mat _current_image;
  cv::Mat _current_mask;

  c_output_frame_writer _processed_file_writer;
};

#endif /* __c_generic_image_processor_h__ */
