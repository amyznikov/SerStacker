/*
 * c_input_options.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_input_options_h__
#define __c_input_options_h__

#include <opencv2/opencv.hpp>
#include "c_data_frame.h"
#include <core/proc/white_balance/histogram_normalization.h>
#include <core/improc/c_image_processor.h>

struct c_video_input_options
{
  DEBAYER_ALGORITHM debayer_method = DEBAYER_NN2;
  bool enable_color_maxtrix = true;
  bool filter_bad_pixels = false;
  double bad_pixels_variation_threshold = 15;
};


struct c_hdl_input_options
{

};

struct c_input_options
{
  c_video_input_options video;
};



#endif /* __c_input_options_h__ */
