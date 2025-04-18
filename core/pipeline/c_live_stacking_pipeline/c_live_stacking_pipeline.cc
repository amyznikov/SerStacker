/*
 * c_live_stacking_pipeline.cc
 *
 *  Created on: Jul 15, 2023
 *      Author: amyznikov
 */

#include "c_live_stacking_pipeline.h"
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<live_stacking_accumulation_type>()
{
  static const c_enum_member members[] = {

      { live_stacking_accumulation_disable, "disable",
          "Simple average" },

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
  return _input_options;
}

c_live_stacking_input_options& c_live_stacking_pipeline::input_options()
{
  return _input_options;
}

const c_live_stacking_registration_options & c_live_stacking_pipeline::registration_options() const
{
  return registration_options_;
}

c_live_stacking_registration_options & c_live_stacking_pipeline::registration_options()
{
  return registration_options_;
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
  static const auto get_group =
      [](c_config_setting setting, bool save, const std::string & name) {
        return save ? setting.add_group(name) : setting[name];
      };


  c_config_setting section, subsection;

  if( !base::serialize(settings, save) ) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "registration_options")) ) {
    SERIALIZE_OPTION(section, save, registration_options_, enabled);
    SERIALIZE_OPTION(section, save, registration_options_, minimum_image_size);
    SERIALIZE_OPTION(section, save, registration_options_, min_rho);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "accumulation_options")) ) {
    SERIALIZE_OPTION(section, save, accumulation_options_, accumulation_type);
    SERIALIZE_OPTION(section, save, accumulation_options_, ignore_input_mask);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, default_display_type);
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_accumulated_file);
    SERIALIZE_OPTION(section, save, output_options_, output_accumuated_file_name);

    SERIALIZE_OPTION(section, save, output_options_, save_accumulated_video);
    if( (subsection = get_group(section, save, "output_incremental_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, output_accumulated_video_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, display_scale);
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl>& c_live_stacking_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
    POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Image registration", "");
    PIPELINE_CTL(ctrls, registration_options_.enabled, "enabled", "");
    PIPELINE_CTL(ctrls, registration_options_.minimum_image_size, "minimum_image_size", "");
    PIPELINE_CTL(ctrls, registration_options_.min_rho, "min_rho", "");

    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Accumulation options", "");
    PIPELINE_CTL(ctrls, accumulation_options_.accumulation_type, "accumulation method", "");
    PIPELINE_CTL(ctrls, accumulation_options_.ignore_input_mask, "ignore input mask", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
    PIPELINE_CTL(ctrls, output_options_.default_display_type, "display_type", "");
    PIPELINE_CTL(ctrls, output_options_.display_scale, "display_scale", "");
    PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");
    PIPELINE_CTL(ctrls, output_options_.save_accumulated_file, "save_accumuated_file", "");
    PIPELINE_CTLC(ctrls, output_options_.output_accumuated_file_name, "accumuated_file_name", "", _this->output_options_.save_accumulated_file);


    PIPELINE_CTL_GROUP(ctrls, "Save accumulated video", "");
      PIPELINE_CTL(ctrls, output_options_.save_accumulated_video, "save_accumulated_video", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.output_accumulated_video_options,
          (_this->output_options_.save_accumulated_video));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}


bool c_live_stacking_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  double display_scale =
      output_options_.display_scale;

  if( display_scale <= 0 && (display_scale = input_display_scale_) <= 0 ) {
    display_scale = 1;
  }

  lock_guard lock(mutex());

  if( frame_accumulation_ ) {
    return frame_accumulation_->compute(display_frame, display_mask, display_scale);
  }

  cv::Mat image, mask;

  if( aligned_image_.empty() ) {
    image = current_image_;
    mask = current_mask_;
  }
  else {
    image = aligned_image_;
    mask = aligned_mask_;
  }

  if( !image.empty() ) {

    if( display_scale == 1 ) {
      image.copyTo(display_frame);
    }
    else {
      image.convertTo(display_frame,
          image.depth(),
          display_scale);
    }

    mask.copyTo(display_mask);

    return true;
  }

  return false;
}


bool c_live_stacking_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  _output_path =
      create_output_path(output_options_.output_directory);

  frame_accumulation_.reset();
  ecch_.clear();
  current_image_.release();
  reference_image_.release();
  aligned_image_.release();

  input_display_scale_ = -1;

  return true;
}

void c_live_stacking_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  if( output_options_.save_accumulated_file ) {

    cv::Mat image, mask;

    if( true ) {
      lock_guard lock(mutex());
      if( frame_accumulation_ && frame_accumulation_->accumulated_frames() > 0 ) {
        if( !frame_accumulation_->compute(image, mask) ) {
          CF_ERROR("frame_accumulation_->compute() fails, can not save accumulator");
        }
      }
    }

    if( !image.empty() ) {

      const std::string output_file_name =
          generate_output_filename(output_options_.output_accumuated_file_name,
              "accumulator",
              ".tiff");

      if( save_image(image, output_file_name) ) {
        CF_DEBUG("Saved '%s'", output_file_name.c_str());
      }
      else {
        CF_DEBUG("save_image('%s') fails", output_file_name.c_str());
      }
    }
  }


}

bool c_live_stacking_pipeline::run_pipeline()
{
  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails, can not run");
    return false;
  }

  const bool is_live_sequence =
      _input_sequence->is_live();

  if( is_live_sequence ) {
    _total_frames = INT_MAX;
    _processed_frames = 0;
    _accumulated_frames = 0;
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
    _processed_frames = 0;
    _accumulated_frames = 0;

    if( _total_frames < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1. input_sequence_->size()=%d",
          _total_frames, _input_sequence->size());
      return false;
    }

    if( !_input_sequence->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    const bool fOk =
        read_input_frame(_input_sequence,
            _input_options,
            current_image_, current_mask_,
            false,
            false);

    if( !fOk ) {
      CF_DEBUG("read_input_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    if ( current_image_.empty() ) {
      // in case of corrupted ASI frame detection the read_input_frame() returns true with empty output image.
      continue;
    }

    if( input_display_scale_ <= 0 ) {
      input_display_scale_ =
          (1 << _input_sequence->bpp());
    }

    if( _input_options.input_image_processor ) {
      lock_guard lock(mutex());
      if( !_input_options.input_image_processor->process(current_image_, current_mask_) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    ++_accumulated_frames;
  }

  return true;
}

bool c_live_stacking_pipeline::accumulate_image(const cv::Mat & current_image, const cv::Mat & current_mask)
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

  if( accumulation_options_.ignore_input_mask ) {

    if( !frame_accumulation_->add(current_image_, cv::noArray()) ) {
      CF_ERROR("frame_accumulation_->add(image.size = %dx%d) fails",
          current_image_.cols, current_image_.rows);
      return false;
    }

  }
  else {

    if( !frame_accumulation_->add(current_image_, current_mask_) ) {
      CF_ERROR("frame_accumulation_->add(image.size = %dx%d channels=%d mask.size=%dx%d channels=%d) fails",
          current_image_.cols, current_image_.rows, current_image_.channels(),
          current_mask_.cols, current_mask_.rows, current_mask_.channels());
      return false;
    }
  }

  return true;
}

bool c_live_stacking_pipeline::process_current_frame()
{

  cv::Mat image, mask;

  if( registration_options_.enabled ) {

    if( !ecch_.image_transform() ) {

      if( !(image_transform_ = create_image_transfrom(registration_options_)) ) {
        CF_ERROR("create_image_transfrom() fails");
        return false;
      }

      ecch_.set_image_transform(image_transform_.get());
      ecch_.set_minimum_image_size(std::max(8, registration_options_.minimum_image_size));
      ecch_.set_maxlevel(-1);
    }


    if( reference_image_.empty() ) {

      if( current_image_.channels() == 3 ) {
        cv::cvtColor(current_image_, reference_image_, cv::COLOR_BGR2GRAY);
      }
      else {
        current_image_.copyTo(reference_image_);
      }

      current_mask_.copyTo(reference_mask_);
      ecch_.set_reference_image(reference_image_, reference_mask_);
    }

    else {

      if( current_image_.channels() == 1 ) {
        ecch_.align(current_image_, current_mask_);
      }
      else {
        cv::Mat gray;
        cv::cvtColor(current_image_, gray, cv::COLOR_BGR2GRAY);
        ecch_.align(gray, current_mask_);
      }

      if(  true ) {

        const cv::Mat2f current_remap =
            ecch_.create_remap();

        lock_guard lock(mutex());


        cv::remap(current_image_, aligned_image_, current_remap,
            cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        if( current_mask_.empty() ) {
          cv::remap(cv::Mat1b(current_image_.size(), 255), aligned_mask_, current_remap,
              cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        else {
          cv::remap(current_mask_, aligned_mask_, current_remap,
              cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }

        cv::compare(aligned_mask_, 254, current_mask_, cv::CMP_GE);
      }
    }

  }


  if( accumulation_options_.accumulation_type != live_stacking_accumulation_disable ) {

    lock_guard lock(mutex());

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



    if( !registration_options_.enabled || frame_accumulation_->accumulated_frames() < 1 ) {
      image = current_image_;
      mask = current_mask_;
    }
    else if( registration_options_.enabled ) {
      image = aligned_image_;
      mask = aligned_mask_;
    }

    if( !image.empty() ) {

      if( !accumulate_image(current_image_, current_mask_) ) {
        CF_ERROR("accumulate_image() fails");
        return false;
      }

      if ( output_options_.save_accumulated_video ) {

        const bool fOK =
            add_output_writer(accumulated_video_writer_,
                output_options_.output_accumulated_video_options,
                "acc",
                ".ser");

        if( !fOK ) {
          CF_ERROR("Can not open output writer '%s'",
              accumulated_video_writer_.filename().c_str());
          return false;
        }

      }

      frame_accumulation_->compute(image, mask);

      if ( !accumulated_video_writer_.write(image, mask) ) {
        CF_ERROR("accumulated_video_writer_.write() fails");
        return false;
      }

    }
  }

  return true;
}



c_image_transform::sptr c_live_stacking_pipeline::create_image_transfrom(const c_live_stacking_registration_options & opts)
{
  c_image_transform::sptr transform =
      ::create_image_transform(IMAGE_MOTION_TRANSLATION);

  return transform;
}

c_frame_accumulation::ptr c_live_stacking_pipeline::create_frame_accumulation(const cv::Size & image_size, int cn,
    live_stacking_accumulation_type type)
{
  switch (type) {
    case live_stacking_accumulation_average: {
      //return c_frame_weigthed_average::ptr(new c_frame_weigthed_average(image_size, CV_MAKETYPE(CV_32F, cn) , CV_32F));
      return c_frame_weigthed_average::ptr(new c_frame_weigthed_average());
    }
    default:
      CF_ERROR("Unsupported live_stacking_accumulation_type %d requested", type);
      break;
  }

  return nullptr;
}
