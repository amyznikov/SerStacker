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
  _input_mask.copyTo(_current_mask);
}

void c_video_frame::get_output_mask(cv::OutputArray output_mask)
{
  if( output_mask.needed() ) {
    this->_current_mask.copyTo(output_mask);
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
      this->_current_mask.copyTo(output_mask);
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
  if( _current_mask.empty() || mode == SELECTION_MASK_REPLACE ) {
    if( sm.empty() ) {
      _current_mask.release();
    }
    else if( sm.channels() == 1 ) {
      sm.getMat().copyTo(_current_mask);
    }
    else {
      reduce_color_channels(sm, _current_mask, cv::REDUCE_MAX);
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
        cv::bitwise_and(cmask, _current_mask, _current_mask);
        break;
      case SELECTION_MASK_OR:
        cv::bitwise_or(cmask, _current_mask, _current_mask);
        break;
      case SELECTION_MASK_XOR:
        cv::bitwise_xor(cmask, _current_mask, _current_mask);
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

