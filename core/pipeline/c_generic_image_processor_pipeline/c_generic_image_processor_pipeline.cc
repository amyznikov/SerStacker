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
  return _processing_options;
}

c_generic_image_processor_options & c_generic_image_processor_pipeline::processing_options()
{
  return _processing_options;
}

const c_generic_image_processor_output_options & c_generic_image_processor_pipeline::output_options() const
{
  return _output_options;
}

c_generic_image_processor_output_options & c_generic_image_processor_pipeline::output_options()
{
  return _output_options;
}


bool c_generic_image_processor_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  _output_path =
      create_output_path(_output_options.output_directory);

  return true;
}

void c_generic_image_processor_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  if ( _processed_file_writer.is_open() ) {
    CF_DEBUG("Closing '%s'", _processed_file_writer.filename().c_str());
    _processed_file_writer.close();
  }
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
      if( !_input_sequence->read(_current_image, &_current_mask) ) {
        CF_DEBUG("input_sequence_->read() fails");
        return false;
      }
    }

    if( canceled() ) {
      break;
    }

    if( _input_options.input_image_processor ) {

      lock_guard lock(mutex());

      if( !_input_options.input_image_processor->process(_current_image, _current_mask) ) {
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
  if( _processing_options.image_processor && !_processing_options.image_processor->empty() ) {
    if( !_processing_options.image_processor->process(_current_image, _current_mask) ) {
      CF_ERROR("image_processor->process() fails");
      return false;
    }
  }

  if( _output_options.save_processed_frames ) {

    if( !_processed_file_writer.is_open() ) {

      const std::string output_filename =
          generate_output_filename(_output_options.processed_file_options.output_filename,
              "processed",
              ".avi");

      const bool fOk =
          _processed_file_writer.open(output_filename,
              _output_options.processed_file_options.ffmpeg_opts,
              _output_options.processed_file_options.output_image_processor,
              _output_options.processed_file_options.output_pixel_depth,
              _output_options.processed_file_options.save_frame_mapping);

      if( !fOk ) {
        CF_ERROR("output_writer_.open(%s) fails",
            output_filename.c_str());
        return false;
      }
    }

    if( !_processed_file_writer.write(_current_image, _current_mask, false, _input_sequence->current_pos() - 1) ) {
      CF_ERROR("output_writer_.write(%s) fails",
          _processed_file_writer.filename().c_str());
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
    SERIALIZE_IMAGE_PROCESSOR(section, save, _processing_options, image_processor);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);

    SERIALIZE_OPTION(section, save, _output_options, save_processed_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "processed_file_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, processed_file_options);
    }
  }

  return true;
}


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_generic_image_processor_input_options> & ctx)
{
  using S = c_generic_image_processor_input_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_generic_image_processor_options> & ctx)
{
  using S = c_generic_image_processor_options;
  ctlbind(ctls, "image_processor",  ctx(&S::image_processor), "");
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_generic_image_processor_output_options> & ctx)
{
  using S = c_generic_image_processor_output_options;

  ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(ctx));

  ctlbind(ctls, "save_processed_frames",  ctx(&S::save_processed_frames), "");
  ctlbind_group(ctls, ctx(&S::save_processed_frames));
    ctlbind(ctls, ctx(&S::processed_file_options));
  ctlbind_end_group(ctls);
}


const c_ctlist<c_generic_image_processor_pipeline> & c_generic_image_processor_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

     ctlbind_expandable_group(ctls, "1. Input options", "");
       ctlbind(ctls, ctx(&this_class::_input_options));
     ctlbind_end_group(ctls);

     ctlbind_expandable_group(ctls, "2. Image processing", "");
       ctlbind(ctls, ctx(&this_class::_processing_options));
     ctlbind_end_group(ctls);

     ctlbind_expandable_group(ctls, "3. Output options", "");
       ctlbind(ctls, ctx(&this_class::_output_options));
     ctlbind_end_group(ctls);
  }

  return ctls;
}


//const std::vector<c_image_processing_pipeline_ctrl>& c_generic_image_processor_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
//
////  if( ctrls.empty() ) {
////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
////      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, _processing_options.image_processor, "image_processor", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////      PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Processed Frames", "");
////        PIPELINE_CTL(ctrls, _output_options.save_processed_frames, "save_processed_frames", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.processed_file_options,
////            _this->_output_options.save_processed_frames);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////  }
//
//  return ctrls;
//}

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
  p->_processing_options = this->_processing_options;
  p->_output_options = this->_output_options;

  return true;
}

bool c_generic_image_processor_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( display_frame.needed() ) {
    _current_image.copyTo(display_frame);
  }
  if( display_mask.needed() ) {
    _current_mask.copyTo(display_mask);
  }
  return true;
}

