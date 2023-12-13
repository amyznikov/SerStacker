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
  add_display_channel(PIXEL_VALUE, "PIXEL_VALUE",
      "PIXEL_VALUE",
      0, 255);
}

void c_video_frame::getSupportedViewTypes(std::set<ViewType> * viewTypes)
{
  viewTypes->emplace(ViewType_Image);
}

bool c_video_frame::getViewData(ViewType * selectedViewType, int selectedDisplayId,
    cv::OutputArray image,
    cv::OutputArray data,
    cv::OutputArray mask)
{
  * selectedViewType = ViewType_Image;

  if ( image.needed() ) {
    this->image.copyTo(image);
  }

  if ( mask.needed() ) {
    this->mask.copyTo(mask);
  }

  if ( data.needed() ) {
    data.release();
  }

  return true;
}

bool c_video_frame::get_image(int id, cv::OutputArray output_image,
    cv::OutputArray output_mask)
{
  switch (id) {
    case PIXEL_VALUE:
      if( output_image.needed() ) {
        this->image.copyTo(output_image);
      }
      if( output_mask.needed() ) {
        this->mask.copyTo(output_mask);
      }
      break;
    default:
      CF_ERROR("Invalid data item id=%d requested", id);
      return false;
  }

  return true;
}

} /* namespace cloudview */
