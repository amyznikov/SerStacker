/*
 * c_generic_image_processor.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "c_generic_image_processor_pipeline.h"
#include <core/proc/unsharp_mask.h>
#include <core/ssprintf.h>
#include <core/debug.h>
#include <type_traits>
#include <chrono>
#include <thread>


c_generic_image_processor_pipeline::c_generic_image_processor_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const c_generic_image_processor_input_options & c_generic_image_processor_pipeline::input_options() const
{
  return _input_options;
}

c_generic_image_processor_input_options & c_generic_image_processor_pipeline::input_options()
{
  return _input_options;
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

//void c_generic_image_processor_pipeline::set_output_file_name(const std::string & v)
//{
//  output_file_name_ = v;
//}
//
//const std::string & c_generic_image_processor_pipeline::output_file_name() const
//{
//  return output_file_name_;
//}

bool c_generic_image_processor_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  _output_path =
      create_output_path(output_options_.output_directory);

//  if( output_options_.save_processed_frames ) {
//    output_file_name_ =
//        generate_output_filename(output_options_.processed_file_options.output_filename,
//            "processed",
//            ".avi");
//  }

  return true;
}

void c_generic_image_processor_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  if ( processed_file_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", processed_file_writer_.filename().c_str());
    processed_file_writer_.close();
  }

  // sharpness_norm_measure_.reset();
}

bool c_generic_image_processor_pipeline::run_pipeline()
{
  if( !_input_sequence ) {
    CF_ERROR("No input_sequence provided, can not run");
    return false;
  }

  if ( !_input_sequence->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence =
      _input_sequence->is_live();

  if( is_live_sequence ) {
    _total_frames = INT_MAX;
  }
  else {

    const int start_pos =
        std::max(_input_options.start_frame_index, 0);

    const int end_pos =
        _input_options.max_input_frames < 1 ?
            _input_sequence->size() :
            std::min(_input_sequence->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);

    _total_frames = end_pos - start_pos;

    if( _total_frames < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          _total_frames,
          start_pos,
          end_pos,
          _input_sequence->size(),
          _input_options.max_input_frames,
          is_live_sequence);
      return false;
    }

//    if( processing_options_.adjust_sharpness && !compute_averaged_sharpeness(start_pos, end_pos) ) {
//      CF_ERROR("ERROR: compute_average_sharpness() fails");
//      return false;
//    }

    if( !_input_sequence->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  _processed_frames = 0;
  _accumulated_frames = 0;
  for( ; _processed_frames < _total_frames;  ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !_input_sequence->read(current_image_, &current_mask_) ) {
        CF_DEBUG("input_sequence_->read() fails");
        return false;
      }
    }

    if( canceled() ) {
      break;
    }

    if( _input_options.input_image_processor ) {

      lock_guard lock(mutex());

      if( !_input_options.input_image_processor->process(current_image_, current_mask_) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }

      if( canceled() ) {
        break;
      }
    }

    if( true ) {

      lock_guard lock(mutex());
      if( !process_current_frame() ) {
        CF_ERROR("process_current_frame() fails");
        return false;
      }

      ++_accumulated_frames;
    }

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

  return true;
}

bool c_generic_image_processor_pipeline::process_current_frame()
{
  if( processing_options_.image_processor && !processing_options_.image_processor->empty() ) {
    if( !processing_options_.image_processor->process(current_image_, current_mask_) ) {
      CF_ERROR("image_processor->process() fails");
      return false;
    }
  }

  if( output_options_.save_processed_frames ) {

    if( !processed_file_writer_.is_open() ) {

      const std::string output_filename =
          generate_output_filename(output_options_.processed_file_options.output_filename,
              "processed",
              ".avi");

      const bool fOk =
          processed_file_writer_.open(output_filename,
              output_options_.processed_file_options.ffmpeg_opts,
              output_options_.processed_file_options.output_image_processor,
              output_options_.processed_file_options.output_pixel_depth,
              output_options_.processed_file_options.save_frame_mapping);

      if( !fOk ) {
        CF_ERROR("output_writer_.open(%s) fails",
            output_filename.c_str());
        return false;
      }
    }

    if( !processed_file_writer_.write(current_image_, current_mask_, false, _input_sequence->current_pos() - 1) ) {
      CF_ERROR("output_writer_.write(%s) fails",
          processed_file_writer_.filename().c_str());
      return false;
    }
  }

  return true;
}

bool c_generic_image_processor_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    SERIALIZE_IMAGE_PROCESSOR(section, save, processing_options_, image_processor);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_processed_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "processed_file_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, processed_file_options);
    }
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl>& c_generic_image_processor_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, processing_options_.image_processor, "image_processor", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");

      PIPELINE_CTL_GROUP(ctrls, "Save Processed Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_processed_frames, "save_processed_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.processed_file_options,
            _this->output_options_.save_processed_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

bool c_generic_image_processor_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_generic_image_processor_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->processing_options_ = this->processing_options_;
  p->output_options_ = this->output_options_;

  return true;
}

bool c_generic_image_processor_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( display_frame.needed() ) {
    current_image_.copyTo(display_frame);
  }
  if( display_mask.needed() ) {
    current_mask_.copyTo(display_mask);
  }
  return true;
}

