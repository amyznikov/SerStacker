/*
 * c_stereo_calibration_pipeline.cc
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#include "c_stereo_calibration_pipeline.h"
#include <core/settings/opencv_settings.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/io/c_output_frame_writer.h>
#include <core/io/load_image.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<stereo_calibration_input_frame_layout_type>()
{
  static constexpr c_enum_member members[] = {
      { stereo_calibration_frame_layout_horizontal, "horizontal", "" },
      { stereo_calibration_frame_layout_vertical, "vertical", "" },
      { stereo_calibration_frame_layout_separate_sources, "separate_sources", "" },
      { stereo_calibration_frame_layout_horizontal }
  };

  return members;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


c_stereo_calibration_pipeline::c_stereo_calibration_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

c_stereo_calibration_pipeline::~c_stereo_calibration_pipeline()
{
  cancel();
}

c_stereo_calibration_input_options & c_stereo_calibration_pipeline::input_options()
{
  return input_options_;
}

const c_stereo_calibration_input_options & c_stereo_calibration_pipeline::input_options() const
{
  return input_options_;
}

void c_stereo_calibration_pipeline::set_output_directory(const std::string & output_directory)
{
  output_options_.output_directory = output_directory;
}

const std::string & c_stereo_calibration_pipeline::output_directory() const
{
  return output_options_.output_directory;
}

bool c_stereo_calibration_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, input_options_, left_stereo_source);
    SERIALIZE_OPTION(section, save, input_options_, right_stereo_source);
    SERIALIZE_OPTION(section, save, input_options_, start_frame_index);
    SERIALIZE_OPTION(section, save, input_options_, max_input_frames);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
  }

  return c_stereo_calibration::serialize(settings, save);
}

bool c_stereo_calibration_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  lock_guard lock(display_lock_);
  c_stereo_calibration::get_display_image(frame, mask);
  return true;
}

bool c_stereo_calibration_pipeline::read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const
{
  lock_guard lock(display_lock_);

  INSTRUMENT_REGION("");

  enum COLORID colorid = COLORID_UNKNOWN;
  int bpp = 0;


  if ( !source->read(output_image, &colorid, &bpp) ) {
    CF_FATAL("source->read() fails\n");
    return false;
  }

  if ( is_bayer_pattern(colorid) ) {

    DEBAYER_ALGORITHM algo =
        default_debayer_algorithm();

    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, colorid, algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, colorid) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }

        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << bpp)));

        if ( !nninterpolation(output_image, output_image, colorid) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toString(algo));
        return false;
    }
  }
  else if ( colorid == COLORID_OPTFLOW || (output_image.channels() != 4 && output_image.channels() != 2) ) {
    output_mask.release();
  }
  else if( !splitbgra(output_image, output_image, &output_mask) ) {
    output_mask.release();
    return false;
  }

  if( input_options_.enable_color_maxtrix && source->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        source->color_matrix());
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



bool c_stereo_calibration_pipeline::read_stereo_frame()
{
  switch (input_options_.layout_type) {
    case stereo_calibration_frame_layout_separate_sources:

      for( int i = 0; i < 2; ++i ) {

        if( !read_input_frame(input_sources_[i], current_frames_[i], current_masks_[i]) ) {
          CF_ERROR("ERROR: read_input_frame() fails for source %d", i);
          return false;
        }

        if( canceled() ) {
          CF_ERROR("canceled");
          return false;
        }
      }
      break;

    case stereo_calibration_frame_layout_horizontal: {

      cv::Mat image, mask;

      if( !read_input_frame(input_sources_[0], image, mask) ) {
        CF_ERROR("ERROR: read_input_frame() fails");
        return false;
      }

      const cv::Rect roi[2] = {
          cv::Rect(0, 0, image.cols / 2, image.rows),
          cv::Rect(image.cols / 2, 0, image.cols / 2, image.rows)
      };

      if ( !input_options_.swap_cameras ) {
        current_frames_[0] = image(roi[0]);
        current_frames_[1] = image(roi[1]);
        if ( !mask.empty() ) {
          current_masks_[0] = mask(roi[0]);
          current_masks_[1] = mask(roi[1]);
        }
      }
      else {
        current_frames_[0] = image(roi[1]);
        current_frames_[1] = image(roi[0]);
        if ( !mask.empty() ) {
          current_masks_[0] = mask(roi[1]);
          current_masks_[1] = mask(roi[0]);
        }
      }

      break;
    }

    case stereo_calibration_frame_layout_vertical: {

      cv::Mat image, mask;

      if( !read_input_frame(input_sources_[0], image, mask) ) {
        CF_ERROR("ERROR: read_input_frame() fails");
        return false;
      }

      const cv::Rect roi[2] = {
          cv::Rect(0, 0, image.cols, image.rows / 2),
          cv::Rect(0, image.rows / 2, image.cols, image.rows / 2)
      };

      if ( !input_options_.swap_cameras ) {
        current_frames_[0] = image(roi[0]);
        current_frames_[1] = image(roi[1]);
        if ( !mask.empty() ) {
          current_masks_[0] = mask(roi[0]);
          current_masks_[1] = mask(roi[1]);
        }
      }
      else {
        current_frames_[0] = image(roi[1]);
        current_frames_[1] = image(roi[0]);
        if ( !mask.empty() ) {
          current_masks_[0] = mask(roi[1]);
          current_masks_[1] = mask(roi[0]);
        }
      }

      break;
    }

  }

  if( current_frames_[0].size() != current_frames_[1].size() ) {
    CF_ERROR("INPUT ERROR: Left (%dx%d) and right (%dx%d) image sizes not equal.\n"
        "Different image sizes are not yet supported",
        current_frames_[0].cols, current_frames_[0].rows,
        current_frames_[1].cols, current_frames_[1].rows);
    return false;
  }

  if( current_frames_[0].channels() != current_frames_[1].channels() ) {
    CF_ERROR("INPUT ERROR: Left (%d) and right (%d) number of image channels not equal.\n"
        "Different image types are not yet supported",
        current_frames_[0].channels(),
        current_frames_[1].channels());
    return false;
  }

  for( int i = 0; i < 2; ++i ) {

    if( stereo_intrinsics_.camera[i].image_size.empty() ) {
      stereo_intrinsics_.camera[i].image_size =
          current_frames_[i].size();
    }
    else if( stereo_intrinsics_.camera[i].image_size != current_frames_[i].size() ) {
      CF_ERROR("INPUT ERROR: Frame size change (%dx%d) -> (%dx%d) not supported\n",
          stereo_intrinsics_.camera[i].image_size.width, stereo_intrinsics_.camera[i].image_size.height,
          current_frames_[i].cols, current_frames_[i].rows);
      return false;
    }
  }

//  if( image_processing_options_.input_image_processor ) {
//    for( int i = 0; i < 2; ++i ) {
//      cv::Mat &image = current_frames_[i];
//      cv::Mat &mask = current_frame_->masks[i];
//
//      if( !image_processing_options_.input_image_processor->process(image, mask) ) {
//        CF_ERROR("ERROR: input_image_processor->process() fails for stereo frame %d", i);
//        return false;
//      }
//    }
//  }


  return true;
}

void c_stereo_calibration_pipeline::update_display_image()
{
  lock_guard lock(display_lock_);

  accumulated_frames_ = object_points_.size();
  c_stereo_calibration::update_display_image();

  on_accumulator_changed();
}

bool c_stereo_calibration_pipeline::canceled()
{
  return c_image_processing_pipeline::canceled();
}

bool c_stereo_calibration_pipeline::initialize_pipeline()
{
  set_pipeline_stage(stereo_calibration_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  c_stereo_calibration::set_output_intrinsics_filename(
      ssprintf("%s/stereo_intrinsics.%s.yml",
          output_path_.c_str(),
          csequence_name()));

  c_stereo_calibration::set_output_extrinsics_filename(
      ssprintf("%s/stereo_extrinsics.%s.yml",
          output_path_.c_str(),
          csequence_name()));

  if ( !c_stereo_calibration::initialize() ) {
    CF_ERROR("c_stereo_calibration::initialize");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  if ( input_options_.left_stereo_source.empty() ) {
    CF_ERROR("ERROR: No left stereo source specified");
    return false;
  }

  if ( input_options_.layout_type == stereo_calibration_frame_layout_separate_sources ) {
    if ( input_options_.right_stereo_source.empty() ) {
      CF_ERROR("ERROR: No right stereo source specified");
      return false;
    }
  }

  input_sources_[0] = input_sequence_->source(input_options_.left_stereo_source);
  if ( !input_sources_[0] ) {
    CF_ERROR("ERROR: requested left stereo source not found in input sequence: %s",
        input_options_.left_stereo_source.c_str());
    return false;
  }

  if( input_options_.layout_type == stereo_calibration_frame_layout_separate_sources ) {
    input_sources_[1] = input_sequence_->source(input_options_.right_stereo_source);
    if( !input_sources_[1] ) {
      CF_ERROR("ERROR: requested right stereo source not found in input sequence: %s",
          input_options_.right_stereo_source.c_str());
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}

void c_stereo_calibration_pipeline::cleanup_pipeline()
{
  set_pipeline_stage(stereo_calibration_finishing);

  close_input_source();

  if( true ) {
    lock_guard lock(display_lock_);
    c_stereo_calibration::cleanup();
  }

  base::cleanup_pipeline();

  set_pipeline_stage(stereo_calibration_idle);
}

void c_stereo_calibration_pipeline::close_input_source()
{
  for( int i = 0; i < 2; ++i ) {
    if( input_sources_[i] ) {
      input_sources_[i]->close();
    }
  }
}

bool c_stereo_calibration_pipeline::open_input_source()
{
  if ( !input_sources_[0]->open() ) {
    CF_ERROR("ERROR: can not open input source '%s'", input_sources_[0]->cfilename());
    return false;
  }

  if ( input_options_.layout_type == stereo_calibration_frame_layout_separate_sources ) {
    if ( !input_sources_[1]->open() ) {
      CF_ERROR("ERROR: can not open input source '%s'", input_sources_[1]->cfilename());
      return false;
    }

    if( input_sources_[0]->size() != input_sources_[1]->size() ) {
      CF_ERROR("ERROR: input sources sizes not match: left size=%d right size=%d ",
          input_sources_[0]->size(), input_sources_[1]->size());
      return false;
    }
  }

  return true;
}

bool c_stereo_calibration_pipeline::seek_input_source(int pos)
{
  if( !input_sources_[0]->seek(pos) ) {
    CF_ERROR("ERROR: input_sources_[0]->seek(pos=%d) fails", pos);
    return false;
  }

  if( input_options_.layout_type == stereo_calibration_frame_layout_separate_sources ) {
    if( !input_sources_[1]->seek(pos) ) {
      CF_ERROR("ERROR: input_sources_[1]->seek(pos=%d) fails", pos);
      return false;
    }
  }

  return true;
}


bool c_stereo_calibration_pipeline::run_stereo_calibration()
{
  c_output_frame_writer progress_writer;

  if ( !open_input_source() ) {
    CF_ERROR("ERROR: open_input_source() fails");
    return false;
  }

  const int start_pos =
      std::max(input_options_.start_frame_index, 0);

  const int end_pos =
      input_options_.max_input_frames < 1 ?
          input_sources_[0]->size() :
          std::min(input_sources_[0]->size(),
              input_options_.start_frame_index + input_options_.max_input_frames);


  total_frames_ = end_pos - start_pos;
  processed_frames_ = 0;
  accumulated_frames_ = 0;

  if( total_frames_ < 1 ) {
    CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
        total_frames_);
    return false;
  }

  if( !seek_input_source(start_pos) ) {
    CF_ERROR("ERROR: seek_input_source(start_pos=%d) fails", start_pos);
    return false;
  }

  set_pipeline_stage(stereo_calibration_in_progress);
  set_status_msg("RUNNING ...");

  bool fOk = true;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

    fOk = true;

    if ( canceled() ) {
      break;
    }

    if ( !read_stereo_frame() ) {
      CF_ERROR("read_stereo_frame() fails");
      break;
    }

    if ( canceled() ) {
      break;
    }

    fOk = c_stereo_calibration::process_stereo_frame(current_frames_, current_masks_);
    if ( !fOk || canceled() ) {
      break;
    }

    update_display_image();

    if ( output_options_.save_calibration_progress_video ) {

      if( !progress_writer.is_open() ) {

        std::string output_file_name =
            generate_output_file_name(output_options_.calibration_progress_filename,
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
        CF_ERROR("display_frame_.write() fails");
        return false;
      }

    }

  }

  close_input_source();

  return !canceled();
}

bool c_stereo_calibration_pipeline::write_output_videos()
{
  if( !output_options_.save_rectified_frames &&
      !output_options_.save_stereo_rectified_frames &&
      !output_options_.save_quad_rectified_frames ) {
    return true;
  }

  c_output_frame_writer video_writer[2];
  c_output_frame_writer stereo_writer;
  c_output_frame_writer quad_writer;

  cv::Size sizes[2];
  cv::Size stereo_size;
  cv::Size quad_size;

  if ( !open_input_source() ) {
    CF_ERROR("ERROR: open_input_source() fails");
    return false;
  }

  CF_DEBUG("Saving debug videos...");

  total_frames_ = input_sources_[0]->size();
  processed_frames_ = 0;
  accumulated_frames_ = 0;

  bool fOK;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

    if( canceled() ) {
      break;
    }

    if ( !read_stereo_frame() ) {
      CF_ERROR("read_stereo_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    sizes[0] = current_frames_[0].size();
    sizes[1] = current_frames_[1].size();

    cv::remap(current_frames_[0], current_frames_[0],
        rmaps_[0], cv::noArray(),
        cv::INTER_LINEAR);

    cv::remap(current_frames_[1], current_frames_[1],
        rmaps_[1], cv::noArray(),
        cv::INTER_LINEAR);


    if ( output_options_.save_rectified_frames ) {

      if( !video_writer[0].is_open() ) {

        std::string output_file_name =
            generate_output_file_name(output_options_.rectified_frames_filename,
                "rect-left",
                ".avi");

        fOK =
            video_writer[0].open(output_file_name,
                current_frames_[0].size(),
                current_frames_[0].channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("video_writer[0].open('%s') fails",
              output_file_name.c_str());
          return false;
        }
      }

      if ( !video_writer[0].write( current_frames_[0], cv::noArray(), false, processed_frames_ ) ) {
        CF_ERROR("video_writer[0].write() fails");
        return false;
      }


      if( !video_writer[1].is_open() ) {

        std::string output_file_name =
            generate_output_file_name(output_options_.rectified_frames_filename,
                "rect-right",
                ".avi");

        fOK =
            video_writer[1].open(output_file_name,
                current_frames_[1].size(),
                current_frames_[1].channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("video_writer[1].open('%s') fails",
              output_file_name.c_str());
          return false;
        }
      }

      if ( !video_writer[1].write( current_frames_[1], cv::noArray(), false, processed_frames_ ) ) {
        CF_ERROR("video_writer[1].write() fails");
        return false;
      }
    }


    if ( output_options_.save_stereo_rectified_frames ) {

      if( !stereo_writer.is_open() ) {

        std::string output_file_name =
            generate_output_file_name(output_options_.stereo_rectified_frames_filename,
                "stereo",
                ".avi");

        stereo_size.width = sizes[0].width + sizes[1].width;
        stereo_size.height = std::max(sizes[0].height, sizes[1].height);

        fOK =
            stereo_writer.open(output_file_name,
                stereo_size,
                current_frames_[0].channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("double_writer.open('%s') fails",
              output_file_name.c_str());
          return false;
        }
      }

      const cv::Rect rc0(0, 0, sizes[0].width, sizes[0].height);
      const cv::Rect rc1(sizes[0].width, 0, sizes[1].width, sizes[1].height);

      display_frame_.create(stereo_size, current_frames_[0].type());
      current_frames_[0].copyTo(display_frame_(rc0));
      current_frames_[1].copyTo(display_frame_(rc1));

      if ( !stereo_writer.write( display_frame_, cv::noArray(), false, processed_frames_ ) ) {
        CF_ERROR("stereo_writer.write() fails");
        return false;
      }
    }

    if ( output_options_.save_quad_rectified_frames ) {

      if( !quad_writer.is_open() ) {

        std::string output_file_name =
            generate_output_file_name(output_options_.quad_rectified_frames_filename,
                "quad",
                ".avi");

        quad_size.width = sizes[0].width + sizes[1].width;
        quad_size.height = sizes[0].height + sizes[1].height;

        fOK =
            quad_writer.open(output_file_name,
                quad_size,
                current_frames_[0].channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("quad_writer.open('%s') fails",
              output_file_name.c_str());
          return false;
        }
      }

      const cv::Rect rc0(0, 0, sizes[0].width, sizes[0].height); // tl -> 0
      const cv::Rect rc1(sizes[0].width, 0, sizes[1].width, sizes[1].height); // tr -> 1
      const cv::Rect rc2(0, sizes[0].height, sizes[1].width, sizes[1].height); // bl -> 1
      const cv::Rect rc3(sizes[0].width, sizes[0].height, sizes[1].width, sizes[1].height); // bl -> blend

      display_frame_.create(quad_size, current_frames_[0].type());
      current_frames_[0].copyTo(display_frame_(rc0));
      current_frames_[1].copyTo(display_frame_(rc1));
      current_frames_[1].copyTo(display_frame_(rc2));
      cv::addWeighted(current_frames_[0], 0.5, current_frames_[1], 0.5, 0, display_frame_(rc3));

      if ( !quad_writer.write( display_frame_, cv::noArray(), false, processed_frames_ ) ) {
        CF_ERROR("quad_writer.write() fails");
        return false;
      }
    }

    if( canceled() ) {
      break;
    }
  }


  return true;
}

bool c_stereo_calibration_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  if( !run_stereo_calibration() ) {
    return false;
  }

  if ( !write_output_videos() ) {
    return false;
  }

  return true;
}



