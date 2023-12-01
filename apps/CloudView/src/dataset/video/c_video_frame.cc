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
  items_.emplace_back(c_cloudview_data_item("Image", c_cloudview_data_item::Type::image, IMAGE, "2Image"));
  items_.emplace_back(c_cloudview_data_item("Mask", c_cloudview_data_item::Type::image, MASK, "Mask for image"));
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
