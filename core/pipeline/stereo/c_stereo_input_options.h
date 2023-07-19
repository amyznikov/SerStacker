/*
 * c_stereo_input_options.h
 *
 *  Created on: Jul 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_input_options_h__
#define __c_stereo_input_options_h__

#include <core/io/c_stereo_input.h>
#include <core/pipeline/c_image_processing_pipeline.h>



struct c_stereo_input_options:
    c_image_processing_pipeline_input_options
{
  std::string left_stereo_source;
  std::string right_stereo_source;

  stereo_input_frame_layout_type layout_type =
      stereo_frame_layout_horizontal;

  bool swap_cameras = false;
};

#endif /* __c_stereo_input_options_h__ */
