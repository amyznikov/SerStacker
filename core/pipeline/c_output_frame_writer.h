/*
 * c_output_frame_writer.h
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_output_frame_writer_h__
#define __c_output_frame_writer_h__

#include <core/io/c_ser_file.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/io/save_image.h>
#include <core/proc/pixtype.h>
#include <core/improc/c_image_processor.h>

struct c_output_frame_writer_options
{
  std::string output_filename;
  c_image_processor::sptr output_image_processor;
  PIXEL_DEPTH output_pixel_depth = PIXEL_DEPTH_NO_CHANGE;
  bool save_frame_mapping = false;
};

class c_output_frame_writer
{
public:
  c_output_frame_writer();
  ~c_output_frame_writer();

  const std::string & filename() const;

  bool open(const std::string & filename,
      bool write_frame_mapping = false);

  bool open(const std::string & filename,
      const c_image_processor::sptr & output_image_processor,
      PIXEL_DEPTH output_depth,
      bool write_frame_mapping = false);

  bool write(cv::InputArray currenFrame,
      cv::InputArray currentMask,
      bool with_alpha_mask = false,
      int seqindex = -1);

  bool is_open() const;
  void close();

protected:
  static bool create_output_frame(const cv::Mat & src, const cv::Mat & src_mask,
      cv::Mat & dst, cv::Mat & dst_mask,
      const c_image_processor::sptr & processor,
      PIXEL_DEPTH ddepth);

protected:
  c_ffmpeg_writer ffmpeg;
  c_ser_writer ser;
  cv::Mat tmp;

  enum
  {
    output_type_unknown,
    output_type_video,
    output_type_ser,
    output_type_images,
  } output_type = output_type_unknown;

  std::string output_file_name;
  FILE *frame_mapping_fp = nullptr;
  int current_frame_index = 0;
  int current_input_sequence_index = 0;
  int64_t pts = 0;

  c_image_processor::sptr output_image_processor_;
  PIXEL_DEPTH output_pixel_depth_ = PIXEL_DEPTH_NO_CHANGE;
};


bool load_settings(c_config_setting settings,
    c_output_frame_writer_options * opts);

bool save_settings(c_config_setting settings,
    const c_output_frame_writer_options & opts);

#endif /* __c_output_frame_writer_h__ */
