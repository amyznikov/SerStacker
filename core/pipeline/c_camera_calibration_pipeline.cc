/*
 * c_camera_calibration_pipeline.cc
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 *
 *  Based on /opencv/apps/interactive-calibration
 */

#include "c_camera_calibration_pipeline.h"
#include <core/settings/opencv_settings.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/io/c_output_frame_writer.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <chrono>
#include <thread>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////


c_camera_calibration_pipeline::c_camera_calibration_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

c_camera_calibration_pipeline::~c_camera_calibration_pipeline()
{
  cancel();
}

c_camera_calibration_input_options & c_camera_calibration_pipeline::input_options()
{
  return input_options_;
}

const c_camera_calibration_input_options & c_camera_calibration_pipeline::input_options() const
{
  return input_options_;
}

bool c_camera_calibration_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  lock_guard lock(display_lock_);
  return c_camera_calibration::get_display_image(frame, mask);
}

bool c_camera_calibration_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, input_options_, start_frame_index);
    SERIALIZE_OPTION(section, save, input_options_, max_input_frames);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
  }

  return c_camera_calibration::serialize(settings, save);
}

bool c_camera_calibration_pipeline::read_input_frame(const c_input_sequence::sptr & input_sequence,
    cv::Mat & output_image, cv::Mat & output_mask) const
{
  lock_guard lock(display_lock_);

  INSTRUMENT_REGION("");

  input_sequence->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence->set_auto_apply_color_matrix(false);

  if ( !input_sequence->read(output_image, &output_mask) ) {
    CF_FATAL("input_sequence->read() fails\n");
    return false;
  }

  if ( is_bayer_pattern(input_sequence->colorid()) ) {

    DEBAYER_ALGORITHM algo =
        default_debayer_algorithm();

    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, input_sequence->colorid(), algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }

        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));

        if ( !nninterpolation(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << input_sequence->bpp())));
        }
        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toString(algo));
        return false;
    }
  }

  if( input_options_.enable_color_maxtrix && input_sequence->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        input_sequence->color_matrix());
  }

//  if ( anscombe_.method() != anscombe_none ) {
//    anscombe_.apply(output_image, output_image);
//  }

  if ( !missing_pixel_mask_.empty() ) {

    if ( output_image.size() != missing_pixel_mask_.size() ) {

      CF_ERROR("Invalid input: "
          "frame and bad pixel mask sizes not match:\n"
          "frame size: %dx%d\n"
          "mask size : %dx%d",
          output_image.cols, output_image.rows,
          missing_pixel_mask_.cols, missing_pixel_mask_.rows);

      return false;
    }

    if ( output_mask.empty() ) {
      missing_pixel_mask_.copyTo(output_mask);
    }
    else {
      cv::bitwise_and(output_mask, missing_pixel_mask_,
          output_mask);
    }
  }

  if ( !output_mask.empty() && input_options_.inpaint_missing_pixels ) {
    linear_interpolation_inpaint(output_image, output_mask, output_image);
  }

  return true;
}
//
//void c_camera_calibration_pipeline::update_display_image()
//{
//  CF_DEBUG("c_camera_calibration_pipeline::update_display_image()");
//
//  lock_guard lock(accumulator_lock_);
//
//  accumulated_frames_ = image_points_.size();
//  c_camera_calibration::update_display_image();
//
//  on_accumulator_changed();
//}

bool c_camera_calibration_pipeline::canceled() const
{
  return c_image_processing_pipeline::canceled();
}

bool c_camera_calibration_pipeline::initialize_pipeline()
{
   if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
    return false;
  }

  output_path_ =
      create_output_path(output_options().output_directory);

  c_camera_calibration::set_output_intrinsics_filename(
      ssprintf("%s/camera_intrinsics.%s.yml",
          output_path_.c_str(),
          csequence_name()));

  if ( !c_camera_calibration::initialize() ) {
    CF_ERROR("c_camera_calibration::initialize() fails");
    return false;
  }


  return true;
}

void c_camera_calibration_pipeline::cleanup_pipeline()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
  }

  if( true ) {
    lock_guard lock(display_lock_);
    c_camera_calibration::cleanup();
  }

  base::cleanup_pipeline();
}

bool c_camera_calibration_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  if ( !run_chessboard_corners_collection() ) {
    CF_ERROR("run_chessboard_corners_collection() fails");
    return false;
  }


  if ( calibration_options_.enable_calibration ) {

    CF_DEBUG("update_stereo_calibration()...");

    if ( !c_camera_calibration::update_calibration() ) {
      CF_ERROR("c_camera_calibration::update_stereo_calibration() fails");
      return false;
    }

    if ( !write_output_videos() ) {
      return false;
    }
  }

  CF_DEBUG("leave");

  return  true;
}

bool c_camera_calibration_pipeline::run_chessboard_corners_collection()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  c_output_frame_writer progress_writer;

  if ( !open_input_sequence() ) {
    CF_ERROR("ERROR: open_input_source() fails");
    return false;
  }

  CF_DEBUG("input_sequence->size()=%d",
      input_sequence_->size());

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

  if( !seek_input_sequence(start_pos) ) {
    CF_ERROR("ERROR: seek_input_source(start_pos=%d) fails", start_pos);
    return false;
  }

  // set_pipeline_stage(stereo_calibration_in_progress);
  set_status_msg("RUNNING ...");

  bool fOk = true;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_update() ) {

    if ( is_bad_frame_index(input_sequence_->current_pos())) {
      CF_DEBUG("Skip frame %d as blacklisted", input_sequence_->current_pos());
      input_sequence_->seek(input_sequence_->current_pos() + 1);
      continue;
    }

    if ( canceled() ) {
      break;
    }

    if( !read_input_frame(input_sequence_, current_frame_, current_mask_) ) {
      set_status_msg("read_input_frame() fails");
      break;
    }

    if ( canceled() ) {
      break;
    }

    if ( true ) {

      lock_guard lock(display_lock_);

      if ( !process_current_frame(false) ) {
        CF_ERROR("process_current_frame() fails");
        break;
      }

      accumulated_frames_ =
          object_points_.size();
    }


    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if ( canceled() ) {
      break;
    }

    if( output_options_.save_progress_video ) {

      cv::Mat image, mask;

      if( c_camera_calibration::get_display_image(image, mask) ) {

        if( !progress_writer.is_open() ) {

          std::string output_file_name =
              generate_output_file_name(output_options_.progress_video_filename,
                  "coverage_progress",
                  ".avi");

          bool fOK =
              progress_writer.open(output_file_name,
                  image.size(),
                  image.channels() > 1,
                  false);

          if( !fOK ) {
            CF_ERROR("progress_writer.open('%s') fails",
                output_file_name.c_str());
            return false;
          }
        }

        if( !progress_writer.write(image, cv::noArray(), false, processed_frames_) ) {
          CF_ERROR("progress_writer.write() fails");
          return false;
        }
      }
    }
  }

  close_input_sequence();

  return !canceled();
}

bool c_camera_calibration_pipeline::open_input_sequence()
{
  if ( !input_sequence_->open() ) {
    set_status_msg("ERROR: input_sequence->open() fails");
    return false;
  }
  return true;
}

void c_camera_calibration_pipeline::close_input_sequence()
{
  input_sequence_->close();
}

bool c_camera_calibration_pipeline::seek_input_sequence(int pos)
{
  if ( !input_sequence_->seek(pos) ) {
    CF_ERROR("ERROR: input_sequence->seek(start_pos=%d) fails", pos);
    return false;
  }
  return true;
}

bool c_camera_calibration_pipeline::write_output_videos()
{
  if( !output_options_.save_rectified_frames ) {
    return true;
  }

  CF_DEBUG("update_undistortion_remap()...");
  update_undistortion_remap();

  if( current_undistortion_remap_.empty() ) {
    CF_ERROR("current_undistortion_remap is empty, can not create rectified images");
    return false;
  }

  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }

  CF_DEBUG("Save rectified videos...");

  /////////////////////////////////////////////////////////////////////////////////////////////////

  c_output_frame_writer writer;
  cv::Mat display_frame, display_mask;

  std::string output_file_name =
      generate_output_file_name(output_options_.rectified_frames_filename,
          "rectified",
          ".avi");


  CF_DEBUG("Saving %s...", output_file_name.c_str());

  if( !writer.open(output_file_name, current_frame_.size(), current_frame_.channels() > 1, false) ) {
    CF_ERROR("ERROR: c_video_writer::open('%s') fails", output_file_name.c_str());
  }
  else {

    total_frames_ = input_sequence_->size();
    processed_frames_ = 0;
    accumulated_frames_ = 0;

    for( on_status_update(); processed_frames_ < total_frames_; ++processed_frames_, on_status_update() ) {

      if( !read_input_frame(input_sequence_, current_frame_, current_mask_) ) {
        set_status_msg("read_input_frame() fails");
        break;
      }

      if( canceled() ) {
        break;
      }

      cv::remap(current_frame_, display_frame,
          current_undistortion_remap_, cv::noArray(),
          cv::INTER_LINEAR);

        accumulated_frames_ =
            processed_frames_;

      if( canceled() ) {
        break;
      }

      if( !writer.write(display_frame, cv::noArray(), false, input_sequence_->current_pos() - 1) ) {
        CF_ERROR("ERROR: writer.write() fails. Disk full ?");
        break;
      }

      if( canceled() ) {
        break;
      }

      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }


  close_input_sequence();

  return true;
}
