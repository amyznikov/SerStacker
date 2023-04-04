/*
 * c_generic_image_processor.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "c_generic_image_processor.h"
#include <core/debug.h>


const c_generic_image_processor_options & c_generic_image_processor::processing_options() const
{
  return processing_options_;
}

c_generic_image_processor_options & c_generic_image_processor::processing_options()
{
  return processing_options_;
}

const c_generic_image_processor_output_options & c_generic_image_processor::output_options() const
{
  return output_options_;
}

c_generic_image_processor_output_options & c_generic_image_processor::output_options()
{
  return output_options_;
}

void c_generic_image_processor::set_output_file_name(const std::string & v)
{
  output_file_name_ = v;
}

const std::string & c_generic_image_processor::output_file_name() const
{
  return output_file_name_;
}

bool c_generic_image_processor::initialize()
{
  return true;
}

void c_generic_image_processor::cleanup()
{
  output_writer_.close();
}

bool c_generic_image_processor::process_frame(const cv::Mat & image, const cv::Mat & mask)
{
  if( &image != &current_image_ ) {
    image.copyTo(current_image_);
  }

  if( &mask != &current_mask_ ) {
    mask.copyTo(current_mask_);
  }


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

bool c_generic_image_processor::canceled() const
{
  return false;
}

bool c_generic_image_processor::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

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

bool c_generic_image_processor::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  if( display_frame.needed() ) {
    current_image_.copyTo(display_frame);
  }
  if( display_mask.needed() ) {
    current_mask_.copyTo(display_mask);
  }
  return true;
}

