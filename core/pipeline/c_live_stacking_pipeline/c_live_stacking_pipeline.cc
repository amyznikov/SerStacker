/*
 * c_live_stacking_pipeline.cc
 *
 *  Created on: Jul 15, 2023
 *      Author: amyznikov
 */

#include "c_live_stacking_pipeline.h"
#include <core/ssprintf.h>
#include <core/debug.h>
#include <chrono>
#include <thread>

template<>
const c_enum_member* members_of<live_stacking_accumulation_type>()
{
  static constexpr c_enum_member members[] = {

      { live_stacking_accumulation_average, "average",
          "Simple average" },

//      { live_stacking_accumulation_bayer_average, "bayer_average",
//          "Experimental code for bayer pattern average" },

      { live_stacking_accumulation_average},
  };

  return members;
}


c_live_stacking_pipeline::c_live_stacking_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const c_live_stacking_input_options& c_live_stacking_pipeline::input_options() const
{
  return input_options_;
}

c_live_stacking_input_options& c_live_stacking_pipeline::input_options()
{
  return input_options_;
}

const c_live_stacking_accumulation_options & c_live_stacking_pipeline::accumulation_options() const
{
  return accumulation_options_;
}

c_live_stacking_accumulation_options & c_live_stacking_pipeline::accumulation_options()
{
  return accumulation_options_;
}

const c_live_stacking_output_options & c_live_stacking_pipeline::output_options() const
{
  return output_options_;
}

c_live_stacking_output_options & c_live_stacking_pipeline::output_options()
{
  return output_options_;
}

bool c_live_stacking_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if( !base::serialize(settings, save) ) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, input_options_, start_frame_index);
    SERIALIZE_OPTION(section, save, input_options_, max_input_frames);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "accumulation_options")) ) {
    SERIALIZE_OPTION(section, save, accumulation_options_, accumulation_type);
    SERIALIZE_OPTION(section, save, accumulation_options_, ignore_input_mask);
  }

  return true;
}

bool c_live_stacking_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( frame_accumulation_ ) {
    return frame_accumulation_->compute(display_frame, display_mask);
  }

  return false;
}


bool c_live_stacking_pipeline::initialize_pipeline()
{
  frame_accumulation_.reset();
  return true;
}

void c_live_stacking_pipeline::cleanup_pipeline()
{
}

bool c_live_stacking_pipeline::run_pipeline()
{
  if( !input_sequence_ ) {
    CF_ERROR("No input_sequence provided, can not run");
    return false;
  }

  const bool is_live_sequence =
      input_sequence_->is_live();

  if( is_live_sequence ) {
    total_frames_ = INT_MAX;
    processed_frames_ = 0;
    accumulated_frames_ = 0;
  }
  else {

    const int start_pos =
        std::max(input_options_.start_frame_index, 0);

    const int end_pos =
        input_options_.max_input_frames < 1 ?
            input_sequence_->size() :
            std::min(input_sequence_->size(),
                input_options_.start_frame_index + input_options_.max_input_frames);

    total_frames_ = end_pos - start_pos;
    processed_frames_ = 0;
    accumulated_frames_ = 0;

    if( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
          total_frames_);
      return false;
    }

    if( input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !input_sequence_->read(current_image_, &current_mask_) ) {
        CF_DEBUG("input_sequence_->read() fails");
        break;
      }
    }

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !process_current_frame() ) {
        CF_ERROR("process_current_frame() fails");
        return false;
      }
      accumulated_frames_ =
          processed_frames_;
    }

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

  return true;
}

bool c_live_stacking_pipeline::process_current_frame()
{
  if( !frame_accumulation_ ) {

    frame_accumulation_ =
        create_frame_accumulation(current_image_.size(),
            current_image_.channels(),
            accumulation_options_.accumulation_type);

    if( !frame_accumulation_ ) {
      CF_ERROR("create_frame_accumulation(current_image_.size = %dx%d) fails",
          current_image_.cols, current_image_.rows);
      return false;
    }
  }

  bool fOk =
      frame_accumulation_->add(current_image_,
          accumulation_options_.ignore_input_mask ? cv::noArray() :
              current_mask_);

  if( !fOk ) {
    CF_ERROR("frame_accumulation_->add(current_image_.size = %dx%d) fails",
        current_image_.cols, current_image_.rows);
    return false;
  }

  return true;
}


c_frame_accumulation::ptr c_live_stacking_pipeline::create_frame_accumulation(const cv::Size & image_size, int cn,
    live_stacking_accumulation_type type)
{
  switch (type) {
    case live_stacking_accumulation_average: {
      return c_frame_weigthed_average::ptr(new c_frame_weigthed_average(image_size, CV_MAKETYPE(CV_32F, cn) , CV_32F));
    }
    default:
      CF_ERROR("Unsupported live_stacking_accumulation_type %d requested", type);
      break;
  }

  return nullptr;
}
