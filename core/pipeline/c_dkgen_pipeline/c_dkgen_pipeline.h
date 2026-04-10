/*
 * c_dkgen_pipeline.h
 *
 *  Created on: Apr 10, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dkgen_pipeline_h__
#define __c_dkgen_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/proc/pixtype.h>

struct c_dkgen_pipeline_input_options
{
  int start_frame_index = 0;
  int max_input_frames = -1;
};

struct c_dkgen_pipeline_output_options:
  c_image_processing_pipeline_output_options
{
  std::string output_file_name;
  PIXEL_DEPTH output_depth = PIXEL_DEPTH_32F;
  bool append_timestamp = true;
  bool append_imagesize = true;
  bool append_pixtype = true;
};



class c_dkgen_pipeline :
  public c_image_processing_pipeline
{
public:
  typedef c_dkgen_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_dkgen_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence);
  const std::string & get_class_name() const override;
  static const std::string & class_name();
  static const std::string & tooltip();

  static const c_ctlist<this_class> & getcontrols();
  bool serialize(c_config_setting settings, bool save) override;
  bool copy_parameters(const base::sptr & dst) const override;

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  bool compute_average(cv::OutputArray avgframe, cv::OutputArray avgmask) const;
  std::string generate_output_file_name(const std::string & filesuffix) const;

protected:
  c_dkgen_pipeline_input_options _input_options;
  c_dkgen_pipeline_output_options _output_options;
  cv::Mat _current_image;
  cv::Mat _current_mask;
  cv::Mat _avg_image;
  cv::Mat _avg_mask;
  cv::Mat _white_mask;
  std::string _timestamp;
};

#endif /* __c_dkgen_pipeline_h__ */
