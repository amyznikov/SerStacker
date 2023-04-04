/*
 * c_generic_image_processor.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_generic_image_processor_h__
#define __c_generic_image_processor_h__

#include <core/improc/c_image_processor.h>
#include <core/settings/opencv_settings.h>
#include <core/io/c_output_frame_writer.h>

struct c_generic_image_processor_options
{
  c_image_processor::sptr image_processor;
};

struct c_generic_image_processor_output_options
{
  std::string output_directory;

  bool save_processed_frames;
  std::string processed_frames_filename;
};

class c_generic_image_processor
{
public:
  c_generic_image_processor() = default;
  virtual ~c_generic_image_processor() = default;

  const c_generic_image_processor_options & processing_options() const;
  c_generic_image_processor_options & processing_options() ;

  const c_generic_image_processor_output_options & output_options() const;
  c_generic_image_processor_output_options & output_options();

  void set_output_file_name(const std::string & v);
  const std::string & output_file_name() const;

protected:
  bool initialize();
  void cleanup();
  bool process_frame(const cv::Mat & image, const cv::Mat & mask);

  virtual bool canceled() const;
  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask);

protected:
  c_generic_image_processor_options processing_options_;
  c_generic_image_processor_output_options output_options_;
  std::string output_file_name_;

  cv::Mat current_image_;
  cv::Mat current_mask_;

  c_output_frame_writer output_writer_;
};

#endif /* __c_generic_image_processor_h__ */
