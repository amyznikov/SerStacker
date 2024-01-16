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

//  bool detect_bad_asi_frames = false;
//
//  std::string darkbayer_filename;
//  cv::Mat darkbayer;
//  bool apply_darkbayer = false;
//
//  std::string flatbayer_filename;
//  cv::Mat flatbayer;
//  bool apply_flatbayer = false;
//
//  std::string missing_pixel_mask_filename;
//  cv::Mat missing_pixel_mask;
//  bool missing_pixels_marked_black = true;
//  bool inpaint_missing_pixels = true;
//  bool apply_missing_pixel_mask = false;
//
//
//  c_histogram_normalization_options background_normalization_options;
//  bool enable_bground_normalization = false;
//
//  c_image_processor::sptr input_image_processor;
};


struct c_input_options
{
  c_video_input_options video;
};

//
//bool apply_input_options(const c_data_frame::sptr & dataframe,
//    c_input_options & opts);


#endif /* __c_input_options_h__ */
