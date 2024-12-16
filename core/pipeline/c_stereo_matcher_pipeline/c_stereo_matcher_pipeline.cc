/*
 * c_stereo_matcher_pipeline.cc
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#include "c_stereo_matcher_pipeline.h"
#include <core/io/save_ply.h>
#include <core/proc/colormap.h>
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
}

c_stereo_matcher_pipeline::~c_stereo_matcher_pipeline()
{
  cancel();
}

c_stereo_matcher_input_options & c_stereo_matcher_pipeline::input_options()
{
  return _input_options;
}

const c_stereo_matcher_input_options & c_stereo_matcher_pipeline::input_options() const
{
  return _input_options;
}

c_stereo_matcher_stereo_rectification_options & c_stereo_matcher_pipeline::stereo_rectification_options()
{
  return stereo_rectification_options_;
}

const c_stereo_matcher_stereo_rectification_options & c_stereo_matcher_pipeline::stereo_rectification_options() const
{
  return stereo_rectification_options_;
}

c_stereo_matcher_processing_options & c_stereo_matcher_pipeline::processing_options()
{
  return processing_options_;
}

const c_stereo_matcher_processing_options & c_stereo_matcher_pipeline::processing_options() const
{
  return processing_options_;
}

c_stereo_matcher_image_processing_options & c_stereo_matcher_pipeline::image_processing_options()
{
  return image_processing_options_;
}

const c_stereo_matcher_image_processing_options & c_stereo_matcher_pipeline::image_processing_options() const
{
  return image_processing_options_;
}

c_regular_stereo_matcher& c_stereo_matcher_pipeline::stereo_matcher()
{
  return stereo_matcher_;
}

const c_regular_stereo_matcher& c_stereo_matcher_pipeline::stereo_matcher() const
{
  return stereo_matcher_;
}

c_stereo_matcher_output_options & c_stereo_matcher_pipeline::output_options()
{
  return output_options_;
}

const c_stereo_matcher_output_options & c_stereo_matcher_pipeline::output_options() const
{
  return output_options_;
}

bool c_stereo_matcher_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if( !base::serialize(settings, save) ) {
    return false;
  }


  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_stereo_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "stereo_rectification")) ) {
    SERIALIZE_OPTION(section, save, stereo_rectification_options_, enabled);
    SERIALIZE_OPTION(section, save, stereo_rectification_options_, camera_intrinsics_yml);
    SERIALIZE_OPTION(section, save, stereo_rectification_options_, camera_extrinsics_yml);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "stereo_matcher")) ) {
    stereo_matcher_.serialize(section, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "processing_options")) ) {
    SERIALIZE_OPTION(section, save, processing_options_, camera_focus);
    SERIALIZE_OPTION(section, save, processing_options_, stereo_baseline);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_depthmaps);
    SERIALIZE_OPTION(section, save, output_options_, depthmap_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_cloud3d_image);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_image_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_cloud3d_ply);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_ply_filename);
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl>& c_stereo_matcher_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_STEREO_INPUT_OPTIONS(ctrls)
      PIPELINE_CTL_GROUP(ctrls, "Input Sequence", "");
        POPULATE_PIPELINE_INPUT_OPTIONS(ctrls);
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Stereo rectification", "");
    PIPELINE_CTL(ctrls, stereo_rectification_options_.enabled, "enabled", "");
    PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, stereo_rectification_options_.camera_intrinsics_yml, "camera_intrinsics_yml", "");
    PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, stereo_rectification_options_.camera_extrinsics_yml, "camera_extrinsics_yml", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Stereo Matcher", "");
      PIPELINE_CTL_STEREO_MATCHER_OPTIONS(ctrls, stereo_matcher_);
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Processing options", "");
    PIPELINE_CTL(ctrls, processing_options_.camera_focus, "camera_focus", "");
    PIPELINE_CTL(ctrls, processing_options_.stereo_baseline, "stereo_baseline", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
    // PIPELINE_CTL_PROCESSOR_SELECTION(ctrls,  image_processing_options_.input_image_processor, "input_image_processor", "");
    PIPELINE_CTL_PROCESSOR_SELECTION(ctrls,  image_processing_options_.remapped_image_processor, "remapped_image_processor", "");
    PIPELINE_CTL_PROCESSOR_SELECTION(ctrls,  image_processing_options_.output_image_processor, "output_image_processor", "");
    PIPELINE_CTL_END_GROUP(ctrls);



    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
    PIPELINE_CTL(ctrls, output_options_.default_display_type, "display_type", "");
    PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");
    PIPELINE_CTL(ctrls, output_options_.save_depthmaps, "save_depthmaps", "");
    PIPELINE_CTLC(ctrls, output_options_.depthmap_filename, "depthmap_filename", "", (_this->output_options_.save_depthmaps));
    PIPELINE_CTL(ctrls, output_options_.save_cloud3d_image, "save_cloud3d_image", "");
    PIPELINE_CTLC(ctrls, output_options_.cloud3d_image_filename, "cloud3d_image_filename", "", (_this->output_options_.save_cloud3d_image));
    PIPELINE_CTL(ctrls, output_options_.save_cloud3d_ply, "save_cloud3d_ply", "");
    PIPELINE_CTLC(ctrls, output_options_.cloud3d_ply_filename, "cloud3d_ply_filename", "", (_this->output_options_.save_cloud3d_ply));
    PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
    PIPELINE_CTLC(ctrls, output_options_.progress_video_filename, "progress_video_filename", "", (_this->output_options_.save_progress_video));
    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

bool c_stereo_matcher_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("Base::initialize_pipeline() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  _output_path =
      create_output_path(output_options().output_directory);

  /////////////////////////////////////////////////////////////////////////////

  if( _input_options.layout_type == stereo_frame_layout_separate_sources ) {
    if( _input_options.left_stereo_source.empty() && _input_options.right_stereo_source.empty() ) {
      CF_ERROR("ERROR: No separate stereo sources video files specified");
      return false;
    }
  }

  if ( _input_options.left_stereo_source.empty() ) {
    CF_WARNING("WARNING: No left stereo source specified, "
        "use continuous input sequence");
  }
  else {
    input_.inputs[0] = _input_sequence->source(_input_options.left_stereo_source);
    if ( !input_.inputs[0] ) {
      CF_ERROR("ERROR: requested left stereo source not found in input sequence: %s",
          _input_options.left_stereo_source.c_str());
      return false;
    }

    if( _input_options.layout_type == stereo_frame_layout_separate_sources ) {
      input_.inputs[1] = _input_sequence->source(_input_options.right_stereo_source);
      if( !input_.inputs[1] ) {
        CF_ERROR("ERROR: requested right stereo source not found in input sequence: %s",
            _input_options.right_stereo_source.c_str());
        return false;
      }
    }
  }

  if( !open_input_source() ) {
    CF_ERROR("open_input_source() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}

void c_stereo_matcher_pipeline::cleanup_pipeline()
{
  close_input_source();
  base::cleanup_pipeline();
}


bool c_stereo_matcher_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  c_output_frame_writer progress_writer;
  c_output_frame_writer depthmaps_writer;
  c_output_frame_writer cloud3d_image_writer;

  //  cv::Mat frames[2];
  //  cv::Mat masks[2];
  cv::Mat1w depthmap;

  if ( _input_sequence->is_live() ) {
    _total_frames = INT_MAX;
    _processed_frames = 0;
    _accumulated_frames = 0;
  }
  else {
    int start_pos, end_pos;

    start_pos =
        std::max(_input_options.start_frame_index, 0);

    if ( input_.inputs[0] ) {
      end_pos =
        _input_options.max_input_frames < 1 ?
            input_.inputs[0]->size() :
            std::min(input_.inputs[0]->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);
    }
    else {
      end_pos =
        _input_options.max_input_frames < 1 ?
            _input_sequence->size() :
            std::min(_input_sequence->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);

    }

    _total_frames = end_pos - start_pos;
    _processed_frames = 0;
    _accumulated_frames = 0;

    if( _total_frames < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
          _total_frames);
      return false;
    }

    if( !seek_input_source(start_pos) ) {
      CF_ERROR("ERROR: seek_input_source(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  bool fOK = true;


  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    fOK = true;

    if ( canceled() ) {
      break;
    }

    if ( !read_input_source() ) {
      CF_ERROR("read_input_source() fails");
      return false;
    }

    if ( canceled() ) {
      break;
    }

    if( !process_current_frames() ) {
      CF_ERROR("process_current_frames() fails");
      return false;
    }

    _accumulated_frames = _processed_frames;


    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if ( output_options_.save_progress_video ) {

      cv::Mat image, mask;

      if ( get_display_image(image, mask) && !image.empty() ) {

        if( !progress_writer.is_open() ) {

          std::string output_file_name =
              generate_output_filename(output_options_.progress_video_filename,
                  "stereo_matcher_progress",
                  ".avi");

          fOK =
              progress_writer.open(output_file_name);

          if( !fOK ) {
            CF_ERROR("progress_writer.open('%s') fails",
                output_file_name.c_str());
            return false;
          }
        }

        if ( !progress_writer.write(image, cv::noArray(), false, _processed_frames ) ) {
          CF_ERROR("progress_writer.write('%s') fails. image: %dx%d channels=%d deprth=%d",
              progress_writer.filename().c_str(),
              image.cols, image.rows, image.channels(), image.depth());
          return false;
        }
      }
    }

 //   CF_DEBUG("output_options_.save_depthmaps=%d", output_options_.save_depthmaps);

    if ( output_options_.save_depthmaps && !current_disparity_.empty() ) {

      if( !depthmaps_writer.is_open() ) {

        std::string output_file_name =
            generate_output_filename(output_options_.depthmap_filename,
                "stereo_matcher_depthmaps",
                ".ser");

        fOK =
            depthmaps_writer.open(output_file_name);

        if( !fOK ) {
          CF_ERROR("depthmaps_writer.open('%s') fails",
              output_file_name.c_str());
          return false;
        }
      }


      disparity_to_depth(current_disparity_, depthmap,
          processing_options_.camera_focus,
          processing_options_.stereo_baseline);

      if ( !depthmaps_writer.write(depthmap, cv::noArray(), false, _processed_frames ) ) {
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
                generate_output_filename(output_options_.cloud3d_image_filename,
                    "images/cloud3d",
                    ".tiff");

            fOK =
                cloud3d_image_writer.open(output_file_name);

            if( !fOK ) {
              CF_ERROR("cloud3d_image_writer.open('%s') fails",
                  output_file_name.c_str());
              return false;
            }
          }

          if ( !cloud3d_image_writer.write(cloud3d, cv::noArray(), false, _processed_frames ) ) {
            CF_ERROR("cloud3d_image_writer.write() fails");
            return false;
          }
        }


        if( output_options_.save_cloud3d_ply ) {

          const std::string output_file_name =
            ssprintf("%s/cloud3d/%s.cloud.%06d.ply",
                _output_path.c_str(),
                csequence_name(),
                _processed_frames);

          cv::Mat3b colors;

          if ( current_frames_[0].channels() == 3 ) {
            colors = current_frames_[0];
          }
          else {
            cv::cvtColor(current_frames_[0], colors, cv::COLOR_GRAY2BGR);
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

bool c_stereo_matcher_pipeline::open_input_source()
{
  if( input_.inputs[0] ) {
    if( !::open_stereo_source(input_, _input_options.layout_type) ) {
      CF_ERROR("::open_stereo_source() fails");
      return false;
    }
  }
  else {
    if( !_input_sequence->open() ) {
      CF_ERROR("input_sequence_->open() fails");
      return false;
    }
  }
  return true;
}

void c_stereo_matcher_pipeline::close_input_source()
{
  if( input_.inputs[0] ) {
    ::close_stereo_source(input_);
  }
  else {
    _input_sequence->close();
  }
}

bool c_stereo_matcher_pipeline::seek_input_source(int pos)
{
  if( input_.inputs[0] ) {
    if ( !::seek_stereo_source(input_, pos) ) {
      CF_ERROR("::seek_stereo_source(pos=%d) fails", pos);
      return false;
    }
  }
  else {
    if ( !_input_sequence->seek(pos) ) {
      CF_ERROR("input_sequence_->seek(pos=%d) fails", pos);
      return false;
    }
  }

  return true;
}

bool c_stereo_matcher_pipeline::read_input_source()
{
  lock_guard lock(mutex());

  if( input_.inputs[0] ) {

    const bool fok =
        ::read_stereo_source(input_,
            _input_options.layout_type,
            _input_options.swap_cameras,
            _input_options.enable_color_maxtrix,
            current_frames_,
            current_masks_);

    if( !fok ) {
      CF_ERROR("read_stereo_source() fails");
      return false;
    }
  }
  else {
    const bool fok =
        ::read_stereo_frame(_input_sequence,
            _input_options.layout_type,
            _input_options.swap_cameras,
            _input_options.enable_color_maxtrix,
            current_frames_,
            current_masks_);

    if( !fok ) {
      CF_ERROR("read_stereo_source() fails");
      return false;
    }
  }

  return true;
}


bool c_stereo_matcher_pipeline::process_current_frames()
{
  update_stereo_rectification_remap();

  for( int i = 0; i < 2; ++i ) {

    lock_guard lock(mutex());

    if ( rmaps_[i].empty() ) {

      if( _input_options.input_image_processor ) {
        if( !_input_options.input_image_processor->process(current_frames_[i], current_masks_[i]) ) {
          CF_ERROR("_input_options->process(frame_index=%d) fails", i);
          return false;
        }
      }
    }
    else if( !_input_options.input_image_processor ) {

      cv::remap(current_frames_[i], current_frames_[i],
          rmaps_[i], cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REFLECT101);

      if( !current_masks_[i].empty() ) {
        cv::remap(current_masks_[i], current_masks_[i],
            rmaps_[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_REFLECT101);

        cv::compare(current_masks_[i], 250,
            current_masks_[i],
            cv::CMP_GT);
      }
    }
    else {

      if( !_input_options.input_image_processor->process(current_frames_[i], current_masks_[i]) ) {
        CF_ERROR("_input_options->process(frame_index=%d) fails", i);
        return false;
      }

      cv::remap(current_frames_[i], current_frames_[i],
          rmaps_[i], cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REFLECT101);

      if( !current_masks_[i].empty() ) {
        cv::remap(current_masks_[i], current_masks_[i],
            rmaps_[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_REFLECT101);

        cv::compare(current_masks_[i], 250,
            current_masks_[i],
            cv::CMP_GT);
      }
    }

    if ( image_processing_options_.remapped_image_processor ) {
      if( !image_processing_options_.remapped_image_processor->process(current_frames_[i], current_masks_[i]) ) {
        CF_ERROR("remapped_image_processor->process(frame_index=%d) fails", i);
        return false;
      }
    }
  }

  if( canceled() ) {
    return false;
  }

  if( true ) {

    cv::Mat disparity;

    // INSTRUMENT_REGION("stereo_matcher");

    if( !stereo_matcher_.compute(current_frames_[0], current_frames_[1], disparity) ) {
      CF_ERROR("stereo_matcher_.compute() fails");
      return false;
    }

    lock_guard lock(mutex());
    current_disparity_ = disparity;
  }

  return true;
}

bool c_stereo_matcher_pipeline::update_stereo_rectification_remap()
{
  lock_guard lock(mutex());

  if( !stereo_rectification_options_.enabled ) {
    for( int i = 0; i < 2; ++i ) {
      rmaps_[i].release();
    }
  }
  else if( rmaps_[0].empty() || rmaps_[1].empty() ) {

    const std::string & camera_intrinsics_yml =
        stereo_rectification_options_.camera_intrinsics_yml;

    if( camera_intrinsics_yml.empty() ) {
      CF_ERROR("camera_intrinsics_yml not specified");
      return false;
    }

    const std::string & camera_extrinsics_yml =
        stereo_rectification_options_.camera_extrinsics_yml;

    if( camera_extrinsics_yml.empty() ) {
      CF_ERROR("camera_extrinsics_yml not specified");
      return false;
    }

    if( !read_stereo_camera_intrinsics_yml(&stereo_intrinsics_, camera_intrinsics_yml) ) {
      CF_ERROR("read_stereo_camera_intrinsics_yml('%s') fails", camera_intrinsics_yml.c_str());
      return false;
    }

    if( !read_stereo_camera_extrinsics_yml(&stereo_extrinsics_, camera_extrinsics_yml) ) {
      CF_ERROR("read_stereo_camera_extrinsics_yml('%s') fails", camera_extrinsics_yml.c_str());
      return false;
    }

    bool fOK =
        create_stereo_rectification(stereo_intrinsics_.camera[0].image_size,
            stereo_intrinsics_,
            stereo_extrinsics_,
            -1,
            rmaps_,
            &new_intrinsics_,
            &new_extrinsics_,
            R_,
            P_,
            &Q_,
            validRoi_);

    if( !fOK ) {
      CF_ERROR("create_stereo_rectification() fails");
      return false;
    }
  }

  return true;
}

const c_enum_member * c_stereo_matcher_pipeline::get_display_types() const
{
  static const c_enum_member members[] = {
      { DISPLAY_DISPARITY, "DISPARITY", "Disparity frame display" },
      { DISPLAY_QUAD, "QUAD", "Quad frame display" },
      { 0 },
  };

  return members;
}

bool c_stereo_matcher_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( current_frames_[0].empty() || current_frames_[1].empty() ) {
    return false;
  }

  switch (_display_type) {
    case DISPLAY_DISPARITY: {

      if ( display_frame.needed() ) {
        current_disparity_.copyTo(display_frame);
      }

      if ( display_mask.needed() ) {
        display_mask.release();
      }
      break;
    }

    case DISPLAY_QUAD:
    default: {
      const cv::Size sizes[2] = {
           current_frames_[0].size(),
           current_frames_[1].size(),
       };

       const cv::Size totalSize(sizes[0].width + sizes[1].width,
           2 * std::max(sizes[0].height, sizes[1].height));

       const cv::Rect roi[4] = {
           cv::Rect(0, 0, sizes[0].width, sizes[0].height),
           cv::Rect(sizes[0].width, 0, sizes[1].width, sizes[1].height),
           cv::Rect(0, std::max(sizes[0].height, sizes[1].height), sizes[0].width, sizes[0].height),
           cv::Rect(sizes[0].width, std::max(sizes[0].height, sizes[1].height), sizes[1].width, sizes[1].height),
       };

       display_frame.create(totalSize,
           CV_MAKETYPE(current_frames_[0].depth(), 3));

       cv::Mat & display_frame_ =
           display_frame.getMatRef();

       for( int i = 0; i < 2; ++i ) {
         if( current_frames_[i].channels() == display_frame_.channels() ) {
           current_frames_[i].copyTo(display_frame_(roi[i]));
         }
         else {
           cv::cvtColor(current_frames_[i], display_frame_(roi[i]),
               cv::COLOR_GRAY2BGR);
         }
       }

       if( display_frame_.depth() != CV_8U ) {
         display_frame_.convertTo(display_frame_, CV_8U);
       }

       if ( !current_disparity_.empty() ) {

         cv::Mat disp;


         current_disparity_.convertTo(disp, CV_8U,
             255 / std::max(1., stereo_matcher_.currentMaxDisparity()));

         apply_colormap(disp, disp, COLORMAP_TURBO);

         const int r =
             stereo_matcher_.currentReferenceImageIndex();

         const cv::Rect &blend_roi =
             roi[2 + r];

         const cv::Rect &disp_roi =
             roi[2 + !r];

         disp.copyTo(display_frame_(disp_roi));

         if ( current_frames_[r].type() == display_frame_.type() ) {
           cv::addWeighted(disp, 0.5, current_frames_[r], 0.5, 0, display_frame_(blend_roi));
         }
         else if ( current_frames_[r].depth() == display_frame_.depth() ) {
           cv::Mat tmp;
           cv::cvtColor(current_frames_[r], tmp, cv::COLOR_GRAY2BGR);
           cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));
         }
         else if ( current_frames_[r].channels() == display_frame_.channels() ) {
           cv::Mat tmp;
           current_frames_[r].convertTo(tmp, display_frame_.depth());
           cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));

         }
         else {
           cv::Mat tmp;
           cv::cvtColor(current_frames_[r], tmp, cv::COLOR_GRAY2BGR);
           tmp.convertTo(tmp, display_frame_.depth());
           cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));
         }
       }

      if ( display_frame.needed() ) {
        display_frame_.copyTo(display_frame);
      }

      if ( display_mask.needed() ) {
        display_mask.release();
      }

      break;
    }
  }

  return true;
}

