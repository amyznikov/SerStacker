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
  add_display_channel(PIXEL_VALUE, "PIXEL_VALUE",
      "PIXEL_VALUE",
      -1, -1);

  viewTypes_.emplace(DataViewType_Image);
}

void c_video_frame::cleanup()
{
  selection_mask.release();
}

void c_video_frame::get_output_mask(cv::OutputArray output_mask)
{
  if ( output_mask.needed() ) {

    if ( this->selection_mask.empty() ) {
      this->mask.copyTo(mask);
    }
    else if ( this->mask.empty() ) {
      this->selection_mask.copyTo(mask);
    }
    else {
      cv::bitwise_and(this->mask, this->selection_mask, output_mask);
    }
  }

}

bool c_video_frame::get_display_data(DataViewType * selectedViewType, int selectedDisplayId,
    cv::OutputArray image,
    cv::OutputArray data,
    cv::OutputArray mask)
{
  * selectedViewType = DataViewType_Image;

  if ( image.needed() ) {
    this->image.copyTo(image);
  }

  get_output_mask(mask);

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

      get_output_mask(output_mask);

      break;
    default:
      CF_ERROR("Invalid data item id=%d requested", id);
      return false;
  }

  return true;
}

void c_video_frame::update_selection(cv::InputArray sm, SELECTION_MASK_MODE mode)
{
  if( selection_mask.empty() || mode == SELECTION_MASK_REPLACE ) {
    if( sm.empty() ) {
      selection_mask.release();
    }
    else if( sm.channels() == 1 ) {
      sm.getMat().copyTo(selection_mask);
    }
    else {
      reduce_color_channels(sm, selection_mask, cv::REDUCE_MAX);
    }
  }
  else if( !sm.empty() ) {

    cv::Mat cmask;

    if( sm.channels() == 1 ) {
      cmask = sm.getMat();
    }
    else {
      reduce_color_channels(sm, cmask, cv::REDUCE_MAX);
    }

    switch (mode) {
      case SELECTION_MASK_AND:
        cv::bitwise_and(cmask, selection_mask, selection_mask);
        break;
      case SELECTION_MASK_OR:
        cv::bitwise_or(cmask, selection_mask, selection_mask);
        break;
      case SELECTION_MASK_XOR:
        cv::bitwise_xor(cmask, selection_mask, selection_mask);
        break;
      default:
        CF_ERROR("Not implemented selection mode %d", mode);
        break;
    }
  }

}


