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
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

template<>
const c_enum_member* members_of<CAMERA_CALIBRATION_STAGE>()
{
  static constexpr c_enum_member members[] = {
      { camera_calibration_idle, "idle", "" },
      { camera_calibration_initialize, "initialize", "" },
      { camera_calibration_in_progress, "in_progress", "" },
      { camera_calibration_finishing, "finishing", "" },
      { camera_calibration_idle }
  };

  return members;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

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
  lock_guard lock(accumulator_lock_);
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
  lock_guard lock(accumulator_lock_);

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

void c_camera_calibration_pipeline::update_display_image()
{
  CF_DEBUG("c_camera_calibration_pipeline::update_display_image()");

  lock_guard lock(accumulator_lock_);

  accumulated_frames_ = image_points_.size();
  c_camera_calibration::update_display_image();

  on_accumulator_changed();
}

bool c_camera_calibration_pipeline::canceled() const
{
  return c_image_processing_pipeline::canceled();
}

bool c_camera_calibration_pipeline::initialize_pipeline()
{
  set_pipeline_stage(camera_calibration_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
    return false;
  }

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
  set_pipeline_stage(camera_calibration_finishing);

  c_camera_calibration::cleanup();

  base::cleanup_pipeline();

  if ( input_sequence_ ) {
    input_sequence_->close();
  }


  set_pipeline_stage(camera_calibration_idle);
}

bool c_camera_calibration_pipeline::run_pipeline()
{
  c_output_frame_writer progress_writer;

  CF_DEBUG("Starting '%s' ...",
      cname());

  if ( !input_sequence_->open() ) {
    set_status_msg("ERROR: input_sequence->open() fails");
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

  if ( total_frames_ < 1 ) {
    CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1", total_frames_);
    return false;
  }

  if ( !input_sequence_->seek(start_pos) ) {
    CF_ERROR("ERROR: input_sequence->seek(start_pos=%d) fails", start_pos);
    return false;
  }

  set_status_msg("RUNNING ...");

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

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

    CF_DEBUG("c_camera_calibration::process_frame()");
    if ( !c_camera_calibration::process_frame(current_frame_, current_mask_) ) {
      CF_ERROR("c_camera_calibration::process_frame() fails");
      break;
    }

    CF_DEBUG("output_options_.save_progress_video=%d", output_options_.save_progress_video);

    if ( output_options_.save_progress_video ) {

      if( !progress_writer.is_open() ) {

        std::string output_file_name =
            generate_output_file_name(output_options_.progress_video_filename,
                "progress",
                ".avi");

        bool fOK =
            progress_writer.open(output_file_name,
                display_frame_.size(),
                display_frame_.channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("display_frame_.open('%s') fails",
              output_file_name.c_str());
          return false;
        }
      }

      if ( !progress_writer.write( display_frame_, cv::noArray(), false, processed_frames_ ) ) {
        CF_ERROR("progress_writer.write() fails");
        return false;
      }
    }

  }

  input_sequence_->close();

  /////////////////////////////////////////////////////////////////////////////////////////////////

  if( !canceled() && output_options_.save_rectified_frames ) {

    if( current_undistortion_remap_.empty() ) {
      CF_ERROR("current_undistortion_remap is empty, can not create rectified images");
    }
    else {

      c_output_frame_writer writer;

      std::string output_file_name =
          output_options_.rectified_frames_filename;

      if( output_file_name.empty() ) {

        output_file_name =
            ssprintf("%s/%s.rectified.avi",
                output_path_.c_str(),
                csequence_name());
      }
      else {

        std::string file_directory;
        std::string file_name;
        std::string file_suffix;

        split_pathfilename(output_file_name,
            &file_directory,
            &file_name,
            &file_suffix);

        if( file_directory.empty() ) {
          file_directory = output_path_;
        }
        else if( !is_absolute_path(file_directory) ) {
          file_directory =
              ssprintf("%s/%s",
                  output_path_.c_str(),
                  file_directory.c_str());
        }

        if( file_name.empty() ) {
          file_name =
              ssprintf("%s.rectified",
                  csequence_name());
        }

        if( file_suffix.empty() ) {
          file_suffix = ".avi";
        }

        output_file_name =
            ssprintf("%s/%s%s",
                file_directory.c_str(),
                file_name.c_str(),
                file_suffix.c_str());
      }


      CF_DEBUG("Saving %s...", output_file_name.c_str());

      if( !writer.open(output_file_name, current_frame_.size(), current_frame_.channels() > 1, false) ) {
        CF_ERROR("ERROR: c_video_writer::open('%s') fails", output_file_name.c_str());
      }
      else if( !input_sequence_->open() ) {
        CF_ERROR("ERROR: input_sequence_->open() fails");
      }
      else {

        total_frames_ = input_sequence_->size();
        processed_frames_ = 0;
        accumulated_frames_ = 0;

        for( on_status_changed(); processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

          if( !read_input_frame(input_sequence_, current_frame_, current_mask_) ) {
            set_status_msg("read_input_frame() fails");
            break;
          }

          if( canceled() ) {
            break;
          }

          if ( true ) {
            lock_guard lock(accumulator_lock_);

            cv::remap(current_frame_, display_frame_,
                current_undistortion_remap_, cv::noArray(),
                cv::INTER_LINEAR);

            accumulated_frames_ =
                processed_frames_;
          }

          on_accumulator_changed();

          if( canceled() ) {
            break;
          }

          if( !writer.write(display_frame_, cv::noArray(), false, input_sequence_->current_pos() - 1) ) {
            CF_ERROR("ERROR: writer.write() fails. Disk full ?");
            break;
          }

          if( canceled() ) {
            break;
          }
        }
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////

  CF_DEBUG("FINISHED");
  return  true;
}

//bool c_camera_calibration_pipeline::run_chessboard_corners_collection()
//{
//}

