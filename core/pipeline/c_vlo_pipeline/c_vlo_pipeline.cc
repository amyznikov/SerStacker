/*
 * c_vlo_pipeline.cc
 *
 *  Created on: Oct 26, 2023
 *      Author: amyznikov
 */

#include "c_vlo_pipeline.h"
#include <core/ssprintf.h>
#include <core/proc/autoclip.h>
#include <core/proc/colormap.h>
#include <core/io/save_ply.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>


//////////////////////////////

template<class ScanType>
static bool get_cloud3d(const ScanType & scan,
    std::vector<cv::Point3f> & output_points,
    std::vector<cv::Vec3b> & output_colors)
{

  static cv::Mat3b lut;
  if ( lut.empty() ) {

    cv::Mat1b gray(1, 256);
    for ( int i = 0; i < 256; ++i ) {
      gray[0][i] = i;
    }

    apply_colormap(gray,
        lut,
        COLORMAP_TURBO);
  }

//  for ( const cv::Point3f & p : cloud3d ) {
//
//    const int dist =
//        //std::min(255, (int) (cvRound(sqrt(p.x * p.x + p.y * p.y + p.z * p.z))));
//        std::min(255, std::max(0, (int) ((p.z + 3) * 255 / 20)));
//
//    colors.emplace_back(lut[0][dist]);
//  }


  output_points.clear();
  output_colors.clear();
  output_points.reserve(scan.NUM_POINTS);
  output_colors.reserve(scan.NUM_POINTS);

  const float firstVertAngle =
      0.5 * 0.05 * scan.NUM_LAYERS;

  const float tick2deg =
      (float) (0.00000008381903173490870551553291862726);

  const float yawCorrection = 0;

  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

    const auto &slot =
        scan.slot[s];

    const float horizontalAngle =
        slot.angleTicks * tick2deg * CV_PI / 180 + yawCorrection;

    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

      const float verticalAngle =
          firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180;

      const float cos_vert = cos(verticalAngle);
      const float sin_vert = sin(verticalAngle);
      const float cos_hor_cos_vert = cos(horizontalAngle) * cos_vert;
      const float sin_hor_cos_vert = sin(horizontalAngle) * cos_vert;

      for( int e = 0; e < 2/*scan.NUM_ECHOS*/; ++e ) {

        const auto &echo =
            slot.echo[l][e];

        const uint16_t distance =
            echo.dist;

        if( distance > 0 && distance < 65534 ) {

          const float scale = 0.01;
          const float x = scale * distance * cos_hor_cos_vert;
          const float y = scale * distance * sin_hor_cos_vert;
          const float z = scale * distance * sin_vert;
          output_points.emplace_back(x, y, z);

          uint8_t ambient = (uint8_t) (std::max(32., std::min(255., 0.02 * slot.ambient[l])));

          output_colors.emplace_back(cv::Vec3b(ambient, ambient, ambient));
        }
      }
    }
  }

  return true;
}



//////////////////////////////


c_vlo_pipeline::c_vlo_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

bool c_vlo_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, input_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    // SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_cloud3d_ply);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_filename);
  }

  return true;
}


const std::vector<c_image_processing_pipeline_ctrl> & c_vlo_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      // PIPELINE_CTL_BROWSE_FOR_DIRECTORY(ctrls, output_options_.output_directory, "output_directory", "");

      PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
      PIPELINE_CTL(ctrls, output_options_.progress_video_filename, "progress_video_filename", "");

      PIPELINE_CTL(ctrls, output_options_.save_cloud3d_ply, "save cloud3d ply", "");
      PIPELINE_CTL(ctrls, output_options_.cloud3d_filename, "cloud3d_filename", "");

    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

bool c_vlo_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  cv::Mat image;
  constexpr c_vlo_reader::DATA_CHANNEL display_channel =
      c_vlo_reader::DATA_CHANNEL_AMBIENT;

  switch (current_scan_.version) {
    case VLO_VERSION_1:
      image = c_vlo_reader::get_image(current_scan_.scan1, display_channel);
      break;
    case VLO_VERSION_3:
      image = c_vlo_reader::get_image(current_scan_.scan3, display_channel);
      break;
    case VLO_VERSION_5:
      image = c_vlo_reader::get_image(current_scan_.scan5, display_channel);
      break;
  }


  if( !image.empty() ) {
    autoclip(image, cv::noArray(),
        0.5, 99.5,
        0, 255);
    image.convertTo(image,
        CV_8U);
  }

  display_frame.move(image);
  display_mask.release();

  return true;
}

bool c_vlo_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  output_path_ =
      create_output_path(output_options_.output_directory);

//  if ( output_options_.save_processed_frames ) {
//    output_file_name_ =
//        generate_output_filename(output_options_.processed_frames_filename,
//            "processed",
//            ".avi");
//  }

  const int num_sources =
      input_sequence_->sources().size();

  for( int i = 0; i < num_sources; ++i ) {

    const c_input_source::sptr &source =
        input_sequence_->source(i);

    c_vlo_input_source *vlo =
        dynamic_cast<c_vlo_input_source*>(source.get());

    if( !vlo ) {
      CF_ERROR("ERROR: input source %d is not a VLO source", i);
      return false;
    }
  }

  return true;
}

void c_vlo_pipeline::cleanup_pipeline()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
  }

  if ( progress_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", progress_writer_.filename().c_str());
    progress_writer_.close();
  }
}

bool c_vlo_pipeline::run_pipeline()
{
  if( !input_sequence_->open() ) {
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

    if( !input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");
  CF_DEBUG("total_frames_=%d", total_frames_);

  processed_frames_ = 0;
  accumulated_frames_ = 0;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( !input_sequence_->seek(processed_frames_) ) {
      CF_ERROR("input_sequence_->seek(pos=%d) fails: %s",
          processed_frames_, strerror(errno));
      return false;
    }

    c_vlo_input_source *vlo =
        dynamic_cast<c_vlo_input_source*>(input_sequence_->current_source().get());
    if( !vlo ) {
      CF_ERROR("dynamic_cast<c_vlo_input_source*>(current_source) fails");
      return false;
    }

    if( true ) {
      lock_guard lock(mutex());

      switch ((current_scan_.version = vlo->version())) {
        case VLO_VERSION_1:
          if( !vlo->read(&current_scan_.scan1) ) {
            CF_ERROR("vlo->read(scan1) fails: %s", strerror(errno));
            return false;
          }
          break;
        case VLO_VERSION_3:
          if( !vlo->read(&current_scan_.scan3) ) {
            CF_ERROR("vlo->read(scan3) fails: %s", strerror(errno));
            return false;
          }
          break;
        case VLO_VERSION_5:
          if( !vlo->read(&current_scan_.scan5) ) {
            CF_ERROR("vlo->read(scan5) fails: %s", strerror(errno));
            return false;
          }
          break;

        default:
          CF_ERROR("Unsupported vlo version detected: %d",
              current_scan_.version);
          return false;
      }
    }

    if( canceled() ) {
      break;
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    accumulated_frames_ =
        processed_frames_ + 1;

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  return true;
}


bool c_vlo_pipeline::process_current_frame()
{
  if( !save_progress_video() ) {
    CF_ERROR("save_progress_video() fails");
    return false;
  }

  if( !save_cloud3d_ply() ) {
    CF_ERROR("save_cloud3d_ply() fails");
    return false;
  }


  return true;
}

bool c_vlo_pipeline::save_progress_video()
{
  if( !output_options_.save_progress_video ) {
    return true; // silently ignore
  }


  cv::Mat display_image, display_mask;

  if ( !get_display_image(display_image, display_mask) ) {
    CF_ERROR("get_display_image() fails");
    return false;
  }

  if( !progress_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename(output_options_.progress_video_filename,
            "_progress",
            ".avi");

    bool fOk =
        progress_writer_.open(output_filename,
            display_image.size(),
            display_image.channels() > 1);

    if( !fOk ) {
      CF_ERROR("progress_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !progress_writer_.write(display_image, display_mask) ) {
    CF_ERROR("progress_writer_.write(%s) fails", progress_writer_.filename().c_str());
    return false;
  }

  return true;
}

bool c_vlo_pipeline::save_cloud3d_ply()
{
  if( !output_options_.save_cloud3d_ply) {
    return true; // silently ignore
  }

  std::vector<cv::Point3f> cloud3d;
  std::vector<cv::Vec3b> colors;

  switch (current_scan_.version) {
    case VLO_VERSION_1:
      if( !get_cloud3d(current_scan_.scan1, cloud3d, colors) ) {
        CF_ERROR("get_cloud3d(scan1) fails");
      }
      break;
    case VLO_VERSION_3:
      if( !get_cloud3d(current_scan_.scan3, cloud3d, colors) ) {
        CF_ERROR("get_cloud3d(scan1) fails");
      }
      break;
    case VLO_VERSION_5:
      if( !get_cloud3d(current_scan_.scan5, cloud3d, colors) ) {
        CF_ERROR("get_cloud3d(scan1) fails");
      }
      break;
  }

  const std::string filename =
      ssprintf("%s/cloud3d/%s/cloud3d.%03d.ply",
          output_path_.c_str(),
          input_sequence_->name().c_str(),
          input_sequence_->current_pos() - 1);

  if ( !save_ply(cloud3d, colors, filename) ) {
    CF_ERROR("save_ply('%s') fails", filename.c_str());
    return false;
  }

  return true;
}
