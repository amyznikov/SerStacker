/*
 * c_video_frame.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "c_video_frame.h"
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

c_video_frame::c_video_frame()
{
  add_image_display("PIXEL_VALUE",
      "PIXEL_VALUE",
      -1, -1);

  _display_types.emplace(DisplayType_Image);
}

void c_video_frame::cleanup()
{
  _input_image.copyTo(_current_image);
  _input_mask.copyTo(_selection_mask);
}

void c_video_frame::get_output_mask(cv::OutputArray output_mask)
{
  if( output_mask.needed() ) {
    this->_selection_mask.copyTo(output_mask);
  }
}

bool c_video_frame::get_image(const std::string & display_name,
    cv::OutputArray output_image,
    cv::OutputArray output_mask,
    cv::OutputArray output_data )
{
  if (base::get_image(display_name, output_image, output_mask, output_data)) {
    return true;
  }

  if ( display_name == "PIXEL_VALUE" ) {

    if ( output_image.needed() ) {
      this->_current_image.copyTo(output_image);
    }

    if( output_mask.needed() ) {
      this->_selection_mask.copyTo(output_mask);
    }

    if ( output_data.needed() ) {
      output_data.release();
    }

    return true;
  }

  return false;
}
