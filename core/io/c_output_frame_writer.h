/*
 * c_output_frame_writer.h
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_output_frame_writer_h__
#define __c_output_frame_writer_h__

#include "c_ser_file.h"
#include "c_ffmpeg_file.h"
#include "save_image.h"


class c_output_frame_writer
{
public:
  c_output_frame_writer();
  ~c_output_frame_writer();

  bool is_open() const;
  bool open(const std::string & filename, const cv::Size & frameSize, bool color, bool write_frame_mapping = false);
  bool write(cv::InputArray currenFrame, cv::InputArray currentMask, bool with_alpha_mask, int seqindex);
  void close();

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
};

#endif /* __c_output_frame_writer_h__ */
