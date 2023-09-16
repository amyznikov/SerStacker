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
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  output_path_ =
      create_output_path(output_options_.output_directory);

  if ( output_options_.save_processed_frames ) {
    output_file_name_ =
        generate_output_filename(output_options_.processed_frames_filename,
            "processed",
            ".avi");
  }

  // averaged_sharpeness_ = 0;

  return true;
}

void c_generic_image_processor_pipeline::cleanup_pipeline()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
  }

  if ( output_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", output_writer_.filename().c_str());
    output_writer_.close();
  }

  // sharpness_norm_measure_.reset();
}

bool c_generic_image_processor_pipeline::run_pipeline()
{
  if( !input_sequence_ ) {
    CF_ERROR("No input_sequence provided, can not run");
    return false;
  }

  if ( !input_sequence_->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence =
      input_sequence_->is_live();

  if( is_live_sequence ) {
    total_frames_ = INT_MAX;
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

    if( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          total_frames_,
          start_pos,
          end_pos,
          input_sequence_->size(),
          input_options_.max_input_frames,
          is_live_sequence);
      return false;
    }

//    if( processing_options_.adjust_sharpness && !compute_averaged_sharpeness(start_pos, end_pos) ) {
//      CF_ERROR("ERROR: compute_average_sharpness() fails");
//      return false;
//    }

    if( !input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  processed_frames_ = 0;
  accumulated_frames_ = 0;
  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      current_image_.release();
      current_mask_.release();
      if( !input_sequence_->read(current_image_, &current_mask_) ) {
        CF_DEBUG("input_sequence_->read() fails");
        break;
      }

      if ( true ) {
        double amin, amax;
        cv::minMaxLoc(current_image_, &amin, &amax);
        CF_DEBUG("READ2: amin=%g amax=%g", amin, amax);
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

bool c_generic_image_processor_pipeline::process_current_frame()
{
//  if ( sharpness_norm_measure_  ) {

//    const double sharpen_factor = 2;
//
//    const double current_sharpeness = 0;
//      sharpness_norm_measure_->measure(current_image_, current_mask_);
//
//    const double alpha = 0.99;
//        // sharpen_factor * (1 - current_sharpeness / averaged_sharpeness_);
//
//    CF_DEBUG("current_sharpeness: %g averaged_sharpeness_: %g alpha=%g", current_sharpeness, averaged_sharpeness_, alpha);
//
//    if ( alpha > 0 && alpha < 1 ) {
//
//      double amin, amax, bmin, bmax;
//
//      cv::minMaxLoc(current_image_, &bmin, &bmax);
//
//      unsharp_mask(current_image_, current_image_,
//          sharpness_norm_measure_->sigma(),
//          alpha, -1, -1);
//
//      cv::minMaxLoc(current_image_, &amin, &amax);
//
//      CF_DEBUG("bmin=%g bmax=%g amin=%g amax=%g", bmin, bmax, amin, amax);
//
//    }
//  }


  if ( processing_options_.image_processor && !processing_options_.image_processor->empty() ) {
    if ( !processing_options_.image_processor->process(current_image_, current_mask_) ) {
      CF_ERROR("image_processor->process() fails");
      return false;
    }

    if ( true ) {
      double amin, amax;
      cv::minMaxLoc(current_image_, &amin, &amax);
      CF_DEBUG("PROC: amin=%g amax=%g", amin, amax);
    }

  }

  if( output_options_.save_processed_frames && !output_file_name_.empty() ) {

    if( !output_writer_.is_open() ) {
      if( !output_writer_.open(output_file_name_, current_image_.size(), current_image_.channels() > 1) ) {
        CF_ERROR("output_writer_.open(%s) fails", output_file_name_.c_str());
        return false;
      }
    }

    if ( true ) {
      double amin, amax;
      cv::minMaxLoc(current_image_, &amin, &amax);
      CF_DEBUG("WRITE: amin=%g amax=%g", amin, amax);
    }

    if( !output_writer_.write(current_image_, current_mask_) ) {
      CF_ERROR("output_writer_.write(%s) fails", output_file_name_.c_str());
      return false;
    }
  }

  return true;
}

//
//bool c_generic_image_processor_pipeline::compute_averaged_sharpeness(int start_pos, int end_pos)
//{
//  if( !input_sequence_->seek(start_pos) ) {
//    CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
//    return false;
//  }
//
//  set_status_msg("RUNNING SHARPENESS MEASURE ...");
//
//  sharpness_norm_measure_.reset(new c_sharpness_norm_measure());
//  if( processing_options_.unsharp_sigma > 0 ) {
//    sharpness_norm_measure_->set_sigma(processing_options_.unsharp_sigma);
//  }
//
//
//  averaged_sharpeness_ = 0;
//  processed_frames_ = 0;
//  accumulated_frames_ = 0;
//
//  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {
//
//    if( canceled() ) {
//      break;
//    }
//
//    if( true ) {
//      lock_guard lock(mutex());
//      if( !input_sequence_->read(current_image_, &current_mask_) ) {
//        CF_DEBUG("input_sequence_->read() fails");
//        break;
//      }
//    }
//
//    if ( true ) {
//      double amin, amax;
//      cv::minMaxLoc(current_image_, &amin, &amax);
//      CF_DEBUG("READ:amin=%g amax=%g", amin, amax);
//    }
//
//
//    if( canceled() ) {
//      break;
//    }
//
//    sharpness_norm_measure_->add(current_image_, current_mask_);
//    accumulated_frames_ = processed_frames_;
//
//    if ( true ) {
//      double amin, amax;
//      cv::minMaxLoc(current_image_, &amin, &amax);
//      CF_DEBUG("sharpness_norm: amin=%g amax=%g", amin, amax);
//    }
//
//
//
//    // give chance to GUI thread to call get_display_image()
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//  }
//
//  averaged_sharpeness_ =
//      sharpness_norm_measure_->average();
//
//  CF_DEBUG("LEAVE: averaged_sharpness_ = %g\n",
//      averaged_sharpeness_);
//
//  return true;
//}
//

bool c_generic_image_processor_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, input_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    SERIALIZE_IMAGE_PROCESSOR(section, save, processing_options_, image_processor);
//    SERIALIZE_OPTION(section, save, processing_options_, adjust_sharpness);
//    SERIALIZE_OPTION(section, save, processing_options_, unsharp_sigma);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_processed_frames);
    SERIALIZE_OPTION(section, save, output_options_, processed_frames_filename);
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
//      PIPELINE_CTL(ctrls, processing_options_.adjust_sharpness, "adjust_sharpness", "");
//      PIPELINE_CTLC(ctrls, processing_options_.unsharp_sigma, "unsharp sigma", "", _this-> processing_options_.adjust_sharpness);
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, processing_options_.image_processor, "image_processor", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL(ctrls, output_options_.save_processed_frames, "save_processed_frames", "");
      PIPELINE_CTLC(ctrls, output_options_.processed_frames_filename, "processed_frames_filename", "",
          _this->output_options_.save_processed_frames);
    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
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

