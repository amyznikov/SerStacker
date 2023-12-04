/*
 * c_video_frame.cc
 *
 *  Created on: Nov 26, 2023
 *      Author: amyznikov
 */

#include "c_video_frame.h"
#include <core/debug.h>

namespace cloudview {

c_video_frame::c_video_frame()
{
  add_data_item("Image", IMAGE,
      c_cloudview_data_item::Type::image,
      "Input image");

  add_data_item("Mask", MASK,
      c_cloudview_data_item::Type::image,
      "Input mask");
}

bool c_video_frame::get_image(int id, cv::OutputArray output_image,
    cv::OutputArray output_mask)
{

  switch (id) {
    case IMAGE:
      if( output_image.needed() ) {
        this->image.copyTo(output_image);
      }
      if( output_mask.needed() ) {
        this->mask.copyTo(output_mask);
      }
      break;
    case MASK:
      if( output_image.needed() ) {
        this->mask.copyTo(output_image);
      }
      if( output_mask.needed() ) {
        output_mask.release();
      }
      break;
    default:
      CF_ERROR("Invalid data item id=%d requested", id);
      return false;
  }

  return true;
}

} /* namespace cloudview */
