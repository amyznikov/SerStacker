/*
 * c_running_average_pipeline.cc
 *
 *  Created on: Feb 24, 2024
 *      Author: amyznikov
 */

#include "c_running_average_pipeline.h"
#include <core/io/load_image.h>

c_running_average_pipeline::c_running_average_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

bool c_running_average_pipeline::serialize(c_config_setting settings, bool save)
{
//  static const auto get_group =
//      [](c_config_setting setting, bool save, const std::string & name) {
//        return save ? setting.add_group(name) : setting[name];
//      };

  c_config_setting section, subsection;

  if( !base::serialize(settings, save) ) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, input_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "registration_options")) ) {
    SERIALIZE_OPTION(section, save, registration_options_, enabled);
    SERIALIZE_OPTION(section, save, registration_options_, min_rho);
    SERIALIZE_PROPERTY(section, save, *this, ecc_max_pyramid_level);
    SERIALIZE_PROPERTY(section, save, *this, ecc_noise_level);
    SERIALIZE_PROPERTY(section, save, *this, ecc_support_scale);
    SERIALIZE_PROPERTY(section, save, *this, ecc_normalization_scale);
    SERIALIZE_PROPERTY(section, save, *this, ecc_input_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, *this, ecc_reference_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, *this, ecc_update_multiplier);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "average_options")) ) {
    SERIALIZE_OPTION(section, save, average_options_, running_weight);
  }


  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, default_display_type);
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_accumulated_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_incremental_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, output_accumulated_video_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, display_scale);
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl> & c_running_average_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
    POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Image registration", "");
      PIPELINE_CTL(ctrls, registration_options_.enabled, "enabled", "");
      PIPELINE_CTL(ctrls, registration_options_.min_rho, "min_rho", "");
      PIPELINE_CTLP(ctrls, ecc_support_scale, "ecc_support_scale", "");
      PIPELINE_CTLP(ctrls, ecc_max_pyramid_level, "ecc_max_pyramid_level", "");
      PIPELINE_CTLP(ctrls, ecc_noise_level, "ecc_noise_level", "");
      PIPELINE_CTLP(ctrls, ecc_normalization_scale, "ecc_normalization_scale", "");
      PIPELINE_CTLP(ctrls, ecc_input_smooth_sigma, "ecc_input_smooth_sigma", "");
      PIPELINE_CTLP(ctrls, ecc_reference_smooth_sigma, "ecc_reference_smooth_sigma", "");
      PIPELINE_CTLP(ctrls, ecc_update_multiplier, "ecc_update_multiplier", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Average options", "");
    PIPELINE_CTL(ctrls, average_options_.running_weight, "running_weight", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
    PIPELINE_CTL(ctrls, output_options_.default_display_type, "display_type", "");
    PIPELINE_CTL(ctrls, output_options_.display_scale, "display_scale", "");
    PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");


    PIPELINE_CTL_GROUP(ctrls, "Save accumulated video", "");
      PIPELINE_CTL(ctrls, output_options_.save_accumulated_video, "save_accumulated_video", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.output_accumulated_video_options,
          (_this->output_options_.save_accumulated_video));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

void c_running_average_pipeline::set_ecc_support_scale(int v)
{
  ecc_.set_support_scale(v);
}

int c_running_average_pipeline::ecc_support_scale() const
{
  return ecc_.support_scale();
}

void c_running_average_pipeline::set_ecc_normalization_scale(int v)
{
  ecc_.set_normalization_scale(v);
}

int c_running_average_pipeline::ecc_normalization_scale() const
{
  return ecc_.normalization_scale();
}

void c_running_average_pipeline::set_ecc_input_smooth_sigma(double v)
{
  ecc_.set_input_smooth_sigma(v);
}

double c_running_average_pipeline::ecc_input_smooth_sigma() const
{
  return ecc_.input_smooth_sigma();
}

void c_running_average_pipeline::set_ecc_reference_smooth_sigma(double v)
{
  ecc_.set_reference_smooth_sigma(v);
}

double c_running_average_pipeline::ecc_reference_smooth_sigma() const
{
  return ecc_.reference_smooth_sigma();
}

void c_running_average_pipeline::set_ecc_update_multiplier(double v)
{
  ecc_.set_update_multiplier(v);
}

double c_running_average_pipeline::ecc_update_multiplier() const
{
  return ecc_.update_multiplier();
}


void c_running_average_pipeline::set_ecc_max_pyramid_level(int v)
{
  ecc_.set_max_pyramid_level(v);
}

int c_running_average_pipeline::ecc_max_pyramid_level() const
{
  return ecc_.max_pyramid_level();
}

void c_running_average_pipeline::set_ecc_min_rho(double v)
{
  registration_options_.min_rho = v;
}

double c_running_average_pipeline::ecc_min_rho() const
{
  return registration_options_.min_rho;
}

void c_running_average_pipeline::set_ecc_noise_level(double v)
{
  ecc_.set_noise_level(v);
}

double c_running_average_pipeline::ecc_noise_level() const
{
  return ecc_.noise_level();
}

bool c_running_average_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if ( average_.compute(display_frame, display_mask) ) {
    return true;
  }
  return false;
}

bool c_running_average_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_running_average_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->input_options_ = this->input_options_;
  p->registration_options_ = this->registration_options_;
  p->average_options_ = this->average_options_;
  p->output_options_ = this->output_options_;

  return true;
}


bool c_running_average_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  output_path_ =
      create_output_path(output_options_.output_directory);

  average_.clear();
  current_image_.release();
  current_mask_.release();

  if ( !input_options_.darkbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(input_options_.darkbayer_filename, darkbayer_, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", input_options_.darkbayer_filename.c_str());
      return false;
    }
  }

  if ( !input_options_.flatbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(input_options_.flatbayer_filename, flatbayer_, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", input_options_.flatbayer_filename.c_str());
      return false;
    }
  }



  if ( !input_options_.missing_pixel_mask_filename.empty() ) {

    if ( !load_image(input_options_.missing_pixel_mask_filename, missing_pixel_mask_) ) {
      CF_ERROR("load_image('%s') fails.", input_options_.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( missing_pixel_mask_.type() != CV_8UC1 ) {
      CF_ERROR("Invalid bad pixels mask %s : \nMust be CV_8UC1 type",
          input_options_.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( !input_options_.missing_pixels_marked_black ) {
      cv::invert(missing_pixel_mask_, missing_pixel_mask_);
    }
  }

  CF_DEBUG("Output path='%s'", this->output_path_.c_str());


  return true;
}

void c_running_average_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();
  // average_.clear();
  current_image_.release();
  current_mask_.release();
}

bool c_running_average_pipeline::start_pipeline()
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
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1. input_sequence_->size()=%d",
          total_frames_, input_sequence_->size());
      return false;
    }

    if( !input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  return true;
}

bool c_running_average_pipeline::run_pipeline()
{
  if ( !start_pipeline() ) {
    CF_ERROR("ERROR: start_pipeline() fails");
    return false;
  }

  set_status_msg("RUNNING ...");

  for( ; processed_frames_ < total_frames_; ++processed_frames_, ++accumulated_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    const bool fOk =
        read_input_frame(input_sequence_,
            input_options_,
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

    if( input_options_.input_image_processor ) {
      if( !input_options_.input_image_processor->process(current_image_, current_mask_) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

  }

  return true;
}

bool c_running_average_pipeline::process_current_frame()
{
  cv::Mat image1, mask1, image2, mask2;
  cv::Mat2f rmap;

  bool has_updates = true;

  if( !registration_options_.enabled || average_.accumulated_frames() < 1 ) {

    lock_guard lock(mutex());

    average_.add(current_image_, current_mask_,
        average_options_.running_weight);
  }
  else {

    average_.compute(image1, mask1);

    if( image1.channels() != 1 ) {
      cv::cvtColor(image1, image1, cv::COLOR_BGR2GRAY);
    }

    if( current_image_.channels() != 1 ) {
      cv::cvtColor(current_image_, image2, cv::COLOR_BGR2GRAY);
    }
    else {
      image2 = current_image_;
    }

    ecc_.set_reference_image(image2, mask2);
    if( ecc_.compute(image1, rmap, mask1) ) {
      lock_guard lock(mutex());
      average_.add(current_image_, current_mask_, average_options_.running_weight, &rmap);
    }
    else {
      has_updates = false;
    }
  }

  if ( has_updates && output_options_.save_accumulated_video  ) {

    const bool fOK =
        add_output_writer(accumulated_video_writer_,
            output_options_.output_accumulated_video_options,
            "accw",
            ".ser");

    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          accumulated_video_writer_.filename().c_str());
      return false;
    }

    average_.compute(image1, mask1);

    if( !accumulated_video_writer_.write(image1, mask1) ) {
      CF_ERROR("accumulated_video_writer_.write() fails");
      return false;
    }
  }

  return true;
}

