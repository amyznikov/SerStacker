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

bool serialize_base_stereo_input_options(c_config_setting section, bool save, c_stereo_input_options & opts);


#define POPULATE_PIPELINE_STEREO_INPUT_OPTIONS(ctrls) \
    PIPELINE_CTL(ctrls, _input_options.layout_type, "stereo frame layout", "");\
    PIPELINE_CTL_INPUT_SOURCE_SELECTION(ctrls, _input_options.left_stereo_source, "left stereo source", "", \
        (_this->input_sequence_ && _this->input_sequence_->sources().size() > 0)); \
    PIPELINE_CTL_INPUT_SOURCE_SELECTION(ctrls, _input_options.right_stereo_source, "right stereo source", "",\
        (_this->input_sequence_ && _this->input_sequence_->sources().size() > 1)); \
    PIPELINE_CTL(ctrls, _input_options.swap_cameras, "swap cameras", ""); \


#endif /* __c_stereo_input_options_h__ */
