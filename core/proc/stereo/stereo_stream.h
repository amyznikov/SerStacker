/*
 * stereo_stream.h
 *
 *  Created on: Mar 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __stereo_stream_h__
#define __stereo_stream_h__

#include <opencv2/opencv.hpp>

enum stereo_stream_layout_type {
  stereo_stream_layout_horizontal_split,
  stereo_stream_layout_vertical_split,
  //stereo_stream_layout_interleaved_frames
};


struct c_stereo_stream_options
{
  enum stereo_stream_layout_type layout_type =
      stereo_stream_layout_horizontal_split;

  bool swap_cameras = false;
  bool downscale_panes = false;
};

#endif /* __stereo_stream_h__ */
