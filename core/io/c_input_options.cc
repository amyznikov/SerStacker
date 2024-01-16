/*
 * c_input_options.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "c_input_options.h"
//#include <core/io/video/c_video_frame.h>
//#include <core/io/vlo/c_vlo_frame.h>
//#include <core/io/load_image.h>
//#include <core/proc/bad_pixels.h>
//#include <core/debug.h>
//
//bool apply_input_options(const c_data_frame::sptr & dataframe, c_input_options & opts)
//{
//  if( c_video_frame * video = dynamic_cast<c_video_frame*>(dataframe.get()) ) {
//
//    if( opts.video.filter_bad_pixels && opts.video.bad_pixels_variation_threshold > 0 ) {
//
//      if( !is_bayer_pattern(video->colorid) ) {
//        median_filter_hot_pixels(video->image, opts.video.bad_pixels_variation_threshold, false);
//      }
//      else if( !extract_bayer_planes(video->image, video->image, video->colorid) ) {
//        CF_ERROR("ERROR: extract_bayer_planes() fails");
//      }
//      else {
//        median_filter_hot_pixels(video->image, opts.video.bad_pixels_variation_threshold, true);
//        if( !nninterpolation(video->image, video->image, video->colorid) ) {
//          CF_ERROR("nninterpolation() fails");
//        }
//      }
//    }
//    else if( is_bayer_pattern(video->colorid) ) {
//      debayer(video->image, video->image, video->colorid,
//          opts.video.debayer_method);
//    }
//
//    return true;
//  }
//
//  if( c_vlo_frame * vlo = dynamic_cast<c_vlo_frame*>(dataframe.get()) ) {
//
//  }
//
//  return true;
//}
