/*
 * c_stereo_input_options.h
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_input_h__
#define __c_stereo_input_h__

#include <opencv2/opencv.hpp>
#include <core/io/c_input_sequence.h>


enum stereo_input_frame_layout_type {
  stereo_frame_layout_horizontal,
  stereo_frame_layout_vertical,
  stereo_frame_layout_separate_sources,
};

struct c_stereo_input_source
{
  c_input_source::sptr inputs[2];
};

bool open_stereo_source(c_stereo_input_source & source,
    stereo_input_frame_layout_type layout_type);

void close_stereo_source(c_stereo_input_source & source);

bool seek_stereo_source(c_stereo_input_source & source,
    int pos);

/**
 * FIXME: missing_pixel_mask is not used
 */
bool read_stereo_source(c_stereo_input_source & source,
    stereo_input_frame_layout_type layout_type,
    bool swap_cameras,
    bool enable_color_maxtrix,
    cv::Mat output_frames[2],
    cv::Mat output_masks[2]);

bool read_stereo_frame(const c_input_sequence::sptr & sequence,
    stereo_input_frame_layout_type layout_type,
    bool swap_cameras,
    bool enable_color_maxtrix,
    cv::Mat output_frames[2],
    cv::Mat output_masks[2]);



#endif /* __c_stereo_input_options_h__ */
