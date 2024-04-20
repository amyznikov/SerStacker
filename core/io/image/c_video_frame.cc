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
  add_display_channel("PIXEL_VALUE",
      "PIXEL_VALUE",
      -1, -1);

  display_types_.emplace(DisplayType_Image);
}

void c_video_frame::cleanup()
{
  input_image_.copyTo(current_image_);
  input_mask_.copyTo(current_mask_);
}

void c_video_frame::get_output_mask(cv::OutputArray output_mask)
{
  if( output_mask.needed() ) {
    this->current_mask_.copyTo(output_mask);
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
      current_image_.copyTo(output_image);
    }

    if( output_mask.needed() ) {
      copy_output_mask(current_mask_, output_mask);
    }

    if ( output_data.needed() ) {
      output_data.release();
    }

    return true;
  }

  return false;
}

//
//bool c_video_frame::get_image(int id, cv::OutputArray output_image,
//    cv::OutputArray output_mask)
//{
//  switch (id) {
//    case PIXEL_VALUE:
//
//      if( output_image.needed() ) {
//        this->current_image_.copyTo(output_image);
//      }
//
//      if( output_mask.needed() ) {
//        this->current_mask_.copyTo(output_mask);
//      }
//
//      break;
//
//    default:
//      CF_ERROR("Invalid data item id=%d requested", id);
//      return false;
//  }
//
//  return true;
//}

void c_video_frame::update_selection(cv::InputArray sm, SELECTION_MASK_MODE mode)
{
  if( current_mask_.empty() || mode == SELECTION_MASK_REPLACE ) {
    if( sm.empty() ) {
      current_mask_.release();
    }
    else if( sm.channels() == 1 ) {
      sm.getMat().copyTo(current_mask_);
    }
    else {
      reduce_color_channels(sm, current_mask_, cv::REDUCE_MAX);
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
        cv::bitwise_and(cmask, current_mask_, current_mask_);
        break;
      case SELECTION_MASK_OR:
        cv::bitwise_or(cmask, current_mask_, current_mask_);
        break;
      case SELECTION_MASK_XOR:
        cv::bitwise_xor(cmask, current_mask_, current_mask_);
        break;
      default:
        CF_ERROR("Not implemented selection mode %d", mode);
        break;
    }
  }

}
//
//bool c_video_frame::get_image(const std::string & name, cv::OutputArray output_image, cv::OutputArray output_mask)
//{
//  if( name.empty() ) {
//
//    if( output_image.needed() ) {
//      this->current_image_.copyTo(output_image);
//    }
//
//    if( output_mask.needed() ) {
//      this->current_mask_.copyTo(output_mask);
//    }
//
//    return true;
//  }
//
//  const auto pos =
//      computed_images_.find(name);
//
//  if( pos == computed_images_.end() ) {
//    CF_ERROR("computed_images_.find(name='%s') fails",
//        name.c_str());
//    return false;
//  }
//
//  if( output_image.needed() ) {
//    pos->second.copyTo(output_image);
//  }
//
//  if( output_mask.needed() ) {
//    this->current_mask_.copyTo(output_mask);
//  }
//
//  return true;
//}

//
//void c_video_frame::set_image(const std::string & name, cv::InputArray image, cv::InputArray mask)
//{
//  if( name.empty() ) {
//    image.copyTo(this->current_image_);
//  }
//  else {
//    image.copyTo(computed_images_[name]);
//  }
//}

