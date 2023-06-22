/*
 * c_stereo_matcher_pipeline.cc
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#include "c_stereo_matcher_pipeline.h"
#include <core/io/c_output_frame_writer.h>
#include <core/io/save_ply.h>
#include <chrono>
#include <thread>


static void disparity_to_depth(const cv::Mat1f & disparity, cv::Mat1w & depths, double f, double baseline)
{
  const cv::Size size = disparity.size();

  depths.create(size);
  depths.setTo(0);

  for ( int y = 0; y < size.height; ++y ) {
    for ( int x = 0; x < size.width; ++x ) {
      if ( disparity[y][x] > 0 ) {
        depths[y][x] = baseline * f / disparity[y][x];
      }
    }
  }
}

static void disparity_to_cloud3d(const cv::Mat1f & disparity, cv::Mat3f & cloud3d, double f, double baseline,
    const cv::Matx33d & camera_matrix)
{
  const cv::Size size =
      disparity.size();

  const cv::Matx33d C =
      camera_matrix.inv();

  cloud3d.create(size);
  cloud3d.setTo(0);

  for( int y = 0; y < size.height; ++y ) {
    for( int x = 0; x < size.width; ++x ) {
      if( disparity[y][x] > 0 ) {

        const double depth =
            baseline * f / disparity[y][x];

        cv::Vec3d V = C * cv::Vec3d(x, y, 1);

        V = V * depth / V[2];

        cloud3d[y][x][0] = V[0];
        cloud3d[y][x][1] = V[1];
        cloud3d[y][x][2] = V[2];
      }
    }
  }
}


c_stereo_matcher_pipeline::c_stereo_matcher_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
  input_options_.input_sequence = input_sequence;
}

c_stereo_matcher_pipeline::~c_stereo_matcher_pipeline()
{
  cancel();
}

c_stereo_input_options & c_stereo_matcher_pipeline::input_options()
{
  return input_options_;
}

const c_stereo_input_options & c_stereo_matcher_pipeline::input_options() const
{
  return input_options_;
}

c_stereo_matcher_processing_options & c_stereo_matcher_pipeline::processing_options()
{
  return processing_options_;
}

const c_stereo_matcher_processing_options & c_stereo_matcher_pipeline::processing_options() const
{
  return processing_options_;
}


bool c_stereo_matcher_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, input_options_, layout_type);
    SERIALIZE_OPTION(section, save, input_options_, swap_cameras);
    SERIALIZE_OPTION(section, save, input_options_, left_stereo_source);
    SERIALIZE_OPTION(section, save, input_options_, right_stereo_source);
    SERIALIZE_OPTION(section, save, input_options_, start_frame_index);
    SERIALIZE_OPTION(section, save, input_options_, max_input_frames);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "processing_options")) ) {
    SERIALIZE_OPTION(section, save, processing_options_, camera_focus);
    SERIALIZE_OPTION(section, save, processing_options_, stereo_baseline);
  }

  return c_regular_stereo::serialize(settings, save);
}


bool c_stereo_matcher_pipeline::initialize_pipeline()
{
  if  ( !base::initialize_pipeline() ) {
    CF_ERROR("Base::initialize_pipeline() fails");
    return false;
  }

  if ( !c_regular_stereo::initialize() ) {
    CF_ERROR("c_regular_stereo::initialize() fails");
    return false;
  }

  output_path_ =
      create_output_path(output_options().output_directory);

  /////////////////////////////////////////////////////////////////////////////

  if ( input_options_.left_stereo_source.empty() ) {
    CF_ERROR("ERROR: No left stereo source specified");
    return false;
  }

  if ( input_options_.layout_type == stereo_frame_layout_separate_sources ) {
    if ( input_options_.right_stereo_source.empty() ) {
      CF_ERROR("ERROR: No right stereo source specified");
      return false;
    }
  }

  input_.inputs[0] = input_sequence_->source(input_options_.left_stereo_source);
  if ( !input_.inputs[0] ) {
    CF_ERROR("ERROR: requested left stereo source not found in input sequence: %s",
        input_options_.left_stereo_source.c_str());
    return false;
  }

  if( input_options_.layout_type == stereo_frame_layout_separate_sources ) {
    input_.inputs[1] = input_sequence_->source(input_options_.right_stereo_source);
    if( !input_.inputs[1] ) {
      CF_ERROR("ERROR: requested right stereo source not found in input sequence: %s",
          input_options_.right_stereo_source.c_str());
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}

void c_stereo_matcher_pipeline::cleanup_pipeline()
{
  close_input_source();

  if( true ) {
    lock_guard lock(display_lock_);
    c_regular_stereo::cleanup();
  }

  base::cleanup_pipeline();
}

bool c_stereo_matcher_pipeline::open_input_source()
{
  return ::open_stereo_source(input_, input_options_);
}

void c_stereo_matcher_pipeline::close_input_source()
{
  ::close_stereo_source(input_);
}

bool c_stereo_matcher_pipeline::seek_input_source(int pos)
{
  return ::seek_stereo_source(input_, pos);
}

bool c_stereo_matcher_pipeline::canceled() const
{
  return c_image_processing_pipeline::canceled();
}


bool c_stereo_matcher_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  lock_guard lock(display_lock_);

  if( !c_regular_stereo::get_display_image(frame, cv::noArray()) ) {
    CF_ERROR("c_regular_stereo::get_display_image() fails");
    return false;
  }

  return true;
}


bool c_stereo_matcher_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  c_output_frame_writer progress_writer;
  c_output_frame_writer depthmaps_writer;
  c_output_frame_writer cloud3d_image_writer;

  cv::Mat frames[2];
  cv::Mat masks[2];
  cv::Mat1w depthmap;



  if ( !open_input_source() ) {
    CF_ERROR("ERROR: open_input_source() fails");
    return false;
  }

  const int start_pos =
      std::max(input_options_.start_frame_index, 0);

  const int end_pos =
      input_options_.max_input_frames < 1 ?
          input_.inputs[0]->size() :
          std::min(input_.inputs[0]->size(),
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

  set_status_msg("RUNNING ...");

  bool fOK = true;


  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_update() ) {

    fOK = true;

    if ( canceled() ) {
      break;
    }

    if( !::read_stereo_source(input_, input_options_, frames, masks) ) {
      CF_ERROR("read_stereo_source() fails");
      return false;
    }

    if ( canceled() ) {
      break;
    }

    if ( true ) {

      lock_guard lock(display_lock_);

      if ( !c_regular_stereo::process_stereo_frame(frames, masks) ) {
        CF_ERROR("c_regular_stereo::process_stereo_frame() fails");
        return false;
      }

      accumulated_frames_ = processed_frames_;
    }


    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if ( output_options_.save_progress_video ) {

      cv::Mat image, mask;

      if ( c_regular_stereo::get_display_image(image, mask) ) {

        if( !progress_writer.is_open() ) {

          std::string output_file_name =
              generate_output_file_name(output_options_.progress_video_filename,
                  "stereo_matcher_progress",
                  ".avi");

          fOK =
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

        if ( !progress_writer.write(image, cv::noArray(), false, processed_frames_ ) ) {
          CF_ERROR("progress_writer.write() fails");
          return false;
        }
      }
    }

 //   CF_DEBUG("output_options_.save_depthmaps=%d", output_options_.save_depthmaps);

    if ( output_options_.save_depthmaps && !current_disparity_.empty() ) {

      if( !depthmaps_writer.is_open() ) {

        std::string output_file_name =
            generate_output_file_name(output_options_.depthmap_filename,
                "stereo_matcher_depthmaps",
                ".ser");

        fOK =
            depthmaps_writer.open(output_file_name,
                current_disparity_.size(),
                false,
                false);

        if( !fOK ) {
          CF_ERROR("depthmaps_writer.open('%s') fails",
              output_file_name.c_str());
          return false;
        }
      }


      disparity_to_depth(current_disparity_, depthmap,
          processing_options_.camera_focus,
          processing_options_.stereo_baseline);

      if ( !depthmaps_writer.write(depthmap, cv::noArray(), false, processed_frames_ ) ) {
        CF_ERROR("progress_writer.write() fails");
        return false;
      }

    }

    /////////

    if( output_options_.save_cloud3d_image || output_options_.save_cloud3d_ply ) {
      if( !current_disparity_.empty() ) {

        static const cv::Matx33d kitti_camera_matrix =
            cv::Matx33d(
                7.215377e+02, 0.000000e+00, 6.095593e+02,
                0.000000e+00, 7.215377e+02, 1.728540e+02,
                0.000000e+00, 0.000000e+00, 1.000000e+00);

        cv::Mat3f cloud3d;

        disparity_to_cloud3d(current_disparity_,
            cloud3d,
            processing_options_.camera_focus,
            processing_options_.stereo_baseline,
            kitti_camera_matrix);


        if ( output_options_.save_cloud3d_image ) {
          if( !cloud3d_image_writer.is_open() ) {

            std::string output_file_name =
                generate_output_file_name(output_options_.cloud3d_image_filename,
                    "images/cloud3d",
                    ".tiff");

            fOK =
                cloud3d_image_writer.open(output_file_name,
                    cloud3d.size(),
                    true,
                    false);

            if( !fOK ) {
              CF_ERROR("cloud3d_image_writer.open('%s') fails",
                  output_file_name.c_str());
              return false;
            }
          }

          if ( !cloud3d_image_writer.write(cloud3d, cv::noArray(), false, processed_frames_ ) ) {
            CF_ERROR("cloud3d_image_writer.write() fails");
            return false;
          }
        }


        if( output_options_.save_cloud3d_ply ) {

          const std::string output_file_name =
            ssprintf("%s/cloud3d/%s.cloud.%06d.ply",
                output_path_.c_str(),
                csequence_name(),
                processed_frames_);

          cv::Mat3b colors;

          if ( current_images_[0].channels() == 3 ) {
            colors = current_images_[0];
          }
          else {
            cv::cvtColor(current_images_[0], colors, cv::COLOR_GRAY2BGR);
          }

          if( !save_ply(cloud3d, colors, current_disparity_ > 0, output_file_name) ) {
            CF_ERROR("save_ply('%s') fails",
                output_file_name.c_str());
            return false;
          }
        }
      }
    }



    ////////


  }




  close_input_source();

  CF_DEBUG("LEAVE");

  return true;
}

