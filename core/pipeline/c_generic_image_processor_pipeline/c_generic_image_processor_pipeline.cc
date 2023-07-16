/*
 * c_generic_image_processor.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "c_generic_image_processor_pipeline.h"
#include <core/debug.h>
#include <chrono>
#include <thread>


c_generic_image_processor_pipeline::c_generic_image_processor_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const c_generic_image_processor_input_options & c_generic_image_processor_pipeline::input_options() const
{
  return input_options_;
}

c_generic_image_processor_input_options & c_generic_image_processor_pipeline::input_options()
{
  return input_options_;
}

const c_generic_image_processor_options & c_generic_image_processor_pipeline::processing_options() const
{
  return processing_options_;
}

c_generic_image_processor_options & c_generic_image_processor_pipeline::processing_options()
{
  return processing_options_;
}

const c_generic_image_processor_output_options & c_generic_image_processor_pipeline::output_options() const
{
  return output_options_;
}

c_generic_image_processor_output_options & c_generic_image_processor_pipeline::output_options()
{
  return output_options_;
}

void c_generic_image_processor_pipeline::set_output_file_name(const std::string & v)
{
  output_file_name_ = v;
}

const std::string & c_generic_image_processor_pipeline::output_file_name() const
{
  return output_file_name_;
}

bool c_generic_image_processor_pipeline::initialize_pipeline()
{
  return true;
}

bool c_generic_image_processor_pipeline::run_pipeline()
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

    if( !input_sequence_->seek(start_pos) ) {
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

void c_generic_image_processor_pipeline::cleanup_pipeline()
{
  if ( output_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", output_writer_.filename().c_str());
    output_writer_.close();
  }
}

bool c_generic_image_processor_pipeline::process_current_frame()
{
  if ( processing_options_.image_processor && !processing_options_.image_processor->empty() ) {
    if ( !processing_options_.image_processor->process(current_image_, current_mask_) ) {
      CF_ERROR("image_processor->process() fails");
      return false;
    }
  }

  if( output_options_.save_processed_frames && !output_file_name_.empty() ) {

    if( !output_writer_.is_open() ) {
      if( !output_writer_.open(output_file_name_, current_image_.size(), current_image_.channels() > 1) ) {
        CF_ERROR("output_writer_.open(%s) fails", output_file_name_.c_str());
        return false;
      }
    }

    if( !output_writer_.write(current_image_, current_mask_) ) {
      CF_ERROR("output_writer_.write(%s) fails", output_file_name_.c_str());
      return false;
    }
  }

  return true;
}

bool c_generic_image_processor_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    SERIALIZE_IMAGE_PROCESSOR(section, save, processing_options_, image_processor);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_processed_frames);
    SERIALIZE_OPTION(section, save, output_options_, processed_frames_filename);
  }

  return true;
}

bool c_generic_image_processor_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  if( display_frame.needed() ) {
    current_image_.copyTo(display_frame);
  }
  if( display_mask.needed() ) {
    current_mask_.copyTo(display_mask);
  }
  return true;
}

