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
  return _stereo_rectification_options;
}

const c_stereo_matcher_stereo_rectification_options & c_stereo_matcher_pipeline::stereo_rectification_options() const
{
  return _stereo_rectification_options;
}

c_stereo_matcher_processing_options & c_stereo_matcher_pipeline::processing_options()
{
  return _processing_options;
}

const c_stereo_matcher_processing_options & c_stereo_matcher_pipeline::processing_options() const
{
  return _processing_options;
}

c_stereo_matcher_image_processing_options & c_stereo_matcher_pipeline::image_processing_options()
{
  return _image_processing_options;
}

const c_stereo_matcher_image_processing_options & c_stereo_matcher_pipeline::image_processing_options() const
{
  return _image_processing_options;
}

c_regular_stereo_matcher& c_stereo_matcher_pipeline::stereo_matcher()
{
  return _stereo_matcher;
}

const c_regular_stereo_matcher& c_stereo_matcher_pipeline::stereo_matcher() const
{
  return _stereo_matcher;
}

c_stereo_matcher_output_options & c_stereo_matcher_pipeline::output_options()
{
  return _output_options;
}

const c_stereo_matcher_output_options & c_stereo_matcher_pipeline::output_options() const
{
  return _output_options;
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
    SERIALIZE_OPTION(section, save, _stereo_rectification_options, enabled);
    SERIALIZE_OPTION(section, save, _stereo_rectification_options, camera_intrinsics_yml);
    SERIALIZE_OPTION(section, save, _stereo_rectification_options, camera_extrinsics_yml);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "stereo_matcher")) ) {
    _stereo_matcher.serialize(section, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "processing_options")) ) {
    SERIALIZE_OPTION(section, save, _processing_options, camera_focus);
    SERIALIZE_OPTION(section, save, _processing_options, stereo_baseline);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    SERIALIZE_OPTION(section, save, _output_options, progress_video_filename);
    SERIALIZE_OPTION(section, save, _output_options, save_depthmaps);
    SERIALIZE_OPTION(section, save, _output_options, depthmap_filename);
    SERIALIZE_OPTION(section, save, _output_options, save_cloud3d_image);
    SERIALIZE_OPTION(section, save, _output_options, cloud3d_image_filename);
    SERIALIZE_OPTION(section, save, _output_options, save_cloud3d_ply);
    SERIALIZE_OPTION(section, save, _output_options, cloud3d_ply_filename);
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_matcher_input_options> & ctx)
{
  using S = c_stereo_matcher_input_options;
  ctlbind(ctls, as_base<c_stereo_input_options>(ctx));
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_matcher_stereo_rectification_options> & ctx)
{
  using S = c_stereo_matcher_stereo_rectification_options;
  ctlbind(ctls, "Enable stereo rectification", ctx(&S::enabled), "");
  ctlbind_browse_for_file(ctls, "camera_intrinsics_yml", ctx(&S::camera_intrinsics_yml), "");
  ctlbind_browse_for_file(ctls, "camera_extrinsics_yml", ctx(&S::camera_extrinsics_yml), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_matcher_processing_options> & ctx)
{
  using S = c_stereo_matcher_processing_options;
  ctlbind(ctls, "camera_focus", ctx(&S::camera_focus), "");
  ctlbind(ctls, "stereo_baseline", ctx(&S::stereo_baseline), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_matcher_image_processing_options> & ctx)
{
  using S = c_stereo_matcher_image_processing_options;
  ctlbind(ctls, "remapped_image_processor", ctx(&S::remapped_image_processor), "");
  ctlbind(ctls, "output_image_processor", ctx(&S::output_image_processor), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_matcher_output_options> & ctx)
{
  using S = c_stereo_matcher_output_options;

  ctlbind(ctls, "display_type", ctx(&S::default_display_type), "");
  ctlbind(ctls, "output_directory", ctx(&S::output_directory), "");
  ctlbind(ctls, "save_depthmaps", ctx(&S::save_depthmaps), "");
  ctlbind(ctls, "depthmap_filename", ctx(&S::depthmap_filename), ""); //, (_this->_output_options.save_depthmaps));
  ctlbind(ctls, "save_cloud3d_image", ctx(&S::save_cloud3d_image), "");
  ctlbind(ctls, "cloud3d_image_filename", ctx(&S::cloud3d_image_filename), ""); //, (_this->_output_options.save_cloud3d_image));
  ctlbind(ctls, "save_cloud3d_ply", ctx(&S::save_cloud3d_ply), "");
  ctlbind(ctls, "cloud3d_ply_filename", ctx(&S::cloud3d_ply_filename), ""); //, (_this->_output_options.save_cloud3d_ply));
  ctlbind(ctls, "save_progress_video", ctx(&S::save_progress_video), "");
  ctlbind(ctls, "progress_video_filename", ctx(&S::progress_video_filename), ""); //, (_this->_output_options.save_progress_video));

}

const c_ctlist<c_stereo_matcher_pipeline> & c_stereo_matcher_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Stereo rectification", "");
      ctlbind(ctls, ctx(&this_class::_stereo_rectification_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Stereo Matcher", "");
      ctlbind(ctls, ctx(&this_class::_processing_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Image processing", "");
      ctlbind(ctls, ctx(&this_class::_image_processing_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Output options", "");
      ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}

//const std::vector<c_image_processing_pipeline_ctrl>& c_stereo_matcher_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
//
////  if( ctrls.empty() ) {
////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////      POPULATE_PIPELINE_STEREO_INPUT_OPTIONS(ctrls)
////      PIPELINE_CTL_GROUP(ctrls, "Input Sequence", "");
////        POPULATE_PIPELINE_INPUT_OPTIONS(ctrls);
////      PIPELINE_CTL_END_GROUP(ctrls);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Stereo rectification", "");
////    PIPELINE_CTL(ctrls, _stereo_rectification_options.enabled, "enabled", "");
////    PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, _stereo_rectification_options.camera_intrinsics_yml, "camera_intrinsics_yml", "");
////    PIPELINE_CTL_BROWSE_FOR_EXISTING_FILE(ctrls, _stereo_rectification_options.camera_extrinsics_yml, "camera_extrinsics_yml", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Stereo Matcher", "");
////      PIPELINE_CTL_STEREO_MATCHER_OPTIONS(ctrls, _stereo_matcher);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Processing options", "");
////    PIPELINE_CTL(ctrls, _processing_options.camera_focus, "camera_focus", "");
////    PIPELINE_CTL(ctrls, _processing_options.stereo_baseline, "stereo_baseline", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
////    // PIPELINE_CTL_PROCESSOR_SELECTION(ctrls,  image_processing_options_.input_image_processor, "input_image_processor", "");
////    PIPELINE_CTL_PROCESSOR_SELECTION(ctrls,  _image_processing_options.remapped_image_processor, "remapped_image_processor", "");
////    PIPELINE_CTL_PROCESSOR_SELECTION(ctrls,  _image_processing_options.output_image_processor, "output_image_processor", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////    PIPELINE_CTL(ctrls, _output_options.default_display_type, "display_type", "");
////    PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
////    PIPELINE_CTL(ctrls, _output_options.save_depthmaps, "save_depthmaps", "");
////    PIPELINE_CTLC(ctrls, _output_options.depthmap_filename, "depthmap_filename", "", (_this->_output_options.save_depthmaps));
////    PIPELINE_CTL(ctrls, _output_options.save_cloud3d_image, "save_cloud3d_image", "");
////    PIPELINE_CTLC(ctrls, _output_options.cloud3d_image_filename, "cloud3d_image_filename", "", (_this->_output_options.save_cloud3d_image));
////    PIPELINE_CTL(ctrls, _output_options.save_cloud3d_ply, "save_cloud3d_ply", "");
////    PIPELINE_CTLC(ctrls, _output_options.cloud3d_ply_filename, "cloud3d_ply_filename", "", (_this->_output_options.save_cloud3d_ply));
////    PIPELINE_CTL(ctrls, _output_options.save_progress_video, "save_progress_video", "");
////    PIPELINE_CTLC(ctrls, _output_options.progress_video_filename, "progress_video_filename", "", (_this->_output_options.save_progress_video));
////    PIPELINE_CTL_END_GROUP(ctrls);
////  }
//
//  return ctrls;
//}

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

  if( _input_options.input_source.layout_type == stereo_frame_layout_separate_sources ) {
    if( _input_options.input_source.left_stereo_source.empty() && _input_options.input_source.right_stereo_source.empty() ) {
      CF_ERROR("ERROR: No separate stereo sources video files specified");
      return false;
    }
  }

  if ( _input_options.input_source.left_stereo_source.empty() ) {
    CF_WARNING("WARNING: No left stereo source specified, "
        "use continuous input sequence");
  }
  else {
    _input.inputs[0] = _input_sequence->source(_input_options.input_source.left_stereo_source);
    if ( !_input.inputs[0] ) {
      CF_ERROR("ERROR: requested left stereo source not found in input sequence: %s",
          _input_options.input_source.left_stereo_source.c_str());
      return false;
    }

    if( _input_options.input_source.layout_type == stereo_frame_layout_separate_sources ) {
      _input.inputs[1] = _input_sequence->source(_input_options.input_source.right_stereo_source);
      if( !_input.inputs[1] ) {
        CF_ERROR("ERROR: requested right stereo source not found in input sequence: %s",
            _input_options.input_source.right_stereo_source.c_str());
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

    if ( _input.inputs[0] ) {
      end_pos =
        _input_options.max_input_frames < 1 ?
            _input.inputs[0]->size() :
            std::min(_input.inputs[0]->size(),
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

    if ( _output_options.save_progress_video ) {

      cv::Mat image, mask;

      if ( get_display_image(image, mask) && !image.empty() ) {

        if( !progress_writer.is_open() ) {

          std::string output_file_name =
              generate_output_filename(_output_options.progress_video_filename,
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

    if ( _output_options.save_depthmaps && !_current_disparity.empty() ) {

      if( !depthmaps_writer.is_open() ) {

        std::string output_file_name =
            generate_output_filename(_output_options.depthmap_filename,
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


      disparity_to_depth(_current_disparity, depthmap,
          _processing_options.camera_focus,
          _processing_options.stereo_baseline);

      if ( !depthmaps_writer.write(depthmap, cv::noArray(), false, _processed_frames ) ) {
        CF_ERROR("progress_writer.write() fails");
        return false;
      }
    }

    /////////

    if( _output_options.save_cloud3d_image || _output_options.save_cloud3d_ply ) {
      if( !_current_disparity.empty() ) {

        static const cv::Matx33d kitti_camera_matrix =
            cv::Matx33d(
                7.215377e+02, 0.000000e+00, 6.095593e+02,
                0.000000e+00, 7.215377e+02, 1.728540e+02,
                0.000000e+00, 0.000000e+00, 1.000000e+00);

        cv::Mat3f cloud3d;

        disparity_to_cloud3d(_current_disparity,
            cloud3d,
            _processing_options.camera_focus,
            _processing_options.stereo_baseline,
            kitti_camera_matrix);


        if ( _output_options.save_cloud3d_image ) {
          if( !cloud3d_image_writer.is_open() ) {

            std::string output_file_name =
                generate_output_filename(_output_options.cloud3d_image_filename,
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


        if( _output_options.save_cloud3d_ply ) {

          const std::string output_file_name =
            ssprintf("%s/cloud3d/%s.cloud.%06d.ply",
                _output_path.c_str(),
                csequence_name(),
                _processed_frames);

          cv::Mat3b colors;

          if ( _current_frames[0].channels() == 3 ) {
            colors = _current_frames[0];
          }
          else {
            cv::cvtColor(_current_frames[0], colors, cv::COLOR_GRAY2BGR);
          }

          if( !save_ply(cloud3d, colors, _current_disparity > 0, output_file_name) ) {
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
  if( _input.inputs[0] ) {
    if( !::open_stereo_source(_input, _input_options.input_source.layout_type) ) {
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
  if( _input.inputs[0] ) {
    ::close_stereo_source(_input);
  }
  else {
    _input_sequence->close();
  }
}

bool c_stereo_matcher_pipeline::seek_input_source(int pos)
{
  if( _input.inputs[0] ) {
    if ( !::seek_stereo_source(_input, pos) ) {
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

  if( _input.inputs[0] ) {

    const bool fok =
        ::read_stereo_source(_input,
            _input_options.input_source.layout_type,
            _input_options.input_source.swap_cameras,
            _input_options.enable_color_maxtrix,
            _current_frames,
            _current_masks);

    if( !fok ) {
      CF_ERROR("read_stereo_source() fails");
      return false;
    }
  }
  else {
    const bool fok =
        ::read_stereo_frame(_input_sequence,
            _input_options.input_source.layout_type,
            _input_options.input_source.swap_cameras,
            _input_options.enable_color_maxtrix,
            _current_frames,
            _current_masks);

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
        if( !_input_options.input_image_processor->process(_current_frames[i], _current_masks[i]) ) {
          CF_ERROR("_input_options->process(frame_index=%d) fails", i);
          return false;
        }
      }
    }
    else if( !_input_options.input_image_processor ) {

      cv::remap(_current_frames[i], _current_frames[i],
          rmaps_[i], cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REFLECT101);

      if( !_current_masks[i].empty() ) {
        cv::remap(_current_masks[i], _current_masks[i],
            rmaps_[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_REFLECT101);

        cv::compare(_current_masks[i], 250,
            _current_masks[i],
            cv::CMP_GT);
      }
    }
    else {

      if( !_input_options.input_image_processor->process(_current_frames[i], _current_masks[i]) ) {
        CF_ERROR("_input_options->process(frame_index=%d) fails", i);
        return false;
      }

      cv::remap(_current_frames[i], _current_frames[i],
          rmaps_[i], cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REFLECT101);

      if( !_current_masks[i].empty() ) {
        cv::remap(_current_masks[i], _current_masks[i],
            rmaps_[i], cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_REFLECT101);

        cv::compare(_current_masks[i], 250,
            _current_masks[i],
            cv::CMP_GT);
      }
    }

    if ( _image_processing_options.remapped_image_processor ) {
      if( !_image_processing_options.remapped_image_processor->process(_current_frames[i], _current_masks[i]) ) {
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

    if( !_stereo_matcher.compute(_current_frames[0], _current_frames[1], disparity) ) {
      CF_ERROR("stereo_matcher_.compute() fails");
      return false;
    }

    lock_guard lock(mutex());
    _current_disparity = disparity;
  }

  return true;
}

bool c_stereo_matcher_pipeline::update_stereo_rectification_remap()
{
  lock_guard lock(mutex());

  if( !_stereo_rectification_options.enabled ) {
    for( int i = 0; i < 2; ++i ) {
      rmaps_[i].release();
    }
  }
  else if( rmaps_[0].empty() || rmaps_[1].empty() ) {

    const std::string & camera_intrinsics_yml =
        _stereo_rectification_options.camera_intrinsics_yml;

    if( camera_intrinsics_yml.empty() ) {
      CF_ERROR("camera_intrinsics_yml not specified");
      return false;
    }

    const std::string & camera_extrinsics_yml =
        _stereo_rectification_options.camera_extrinsics_yml;

    if( camera_extrinsics_yml.empty() ) {
      CF_ERROR("camera_extrinsics_yml not specified");
      return false;
    }

    if( !read_stereo_camera_intrinsics_yml(&_stereo_intrinsics, camera_intrinsics_yml) ) {
      CF_ERROR("read_stereo_camera_intrinsics_yml('%s') fails", camera_intrinsics_yml.c_str());
      return false;
    }

    if( !read_stereo_camera_extrinsics_yml(&_stereo_extrinsics, camera_extrinsics_yml) ) {
      CF_ERROR("read_stereo_camera_extrinsics_yml('%s') fails", camera_extrinsics_yml.c_str());
      return false;
    }

    bool fOK =
        create_stereo_rectification(_stereo_intrinsics.camera[0].image_size,
            _stereo_intrinsics,
            _stereo_extrinsics,
            -1,
            rmaps_,
            &_new_intrinsics,
            &_new_extrinsics,
            _R,
            _P,
            &_Q,
            _validRoi);

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

  if( _current_frames[0].empty() || _current_frames[1].empty() ) {
    return false;
  }

  switch (_display_type) {
    case DISPLAY_DISPARITY: {

      if ( display_frame.needed() ) {
        _current_disparity.copyTo(display_frame);
      }

      if ( display_mask.needed() ) {
        display_mask.release();
      }
      break;
    }

    case DISPLAY_QUAD:
    default: {
      const cv::Size sizes[2] = {
           _current_frames[0].size(),
           _current_frames[1].size(),
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
           CV_MAKETYPE(_current_frames[0].depth(), 3));

       cv::Mat & display_frame_ =
           display_frame.getMatRef();

       for( int i = 0; i < 2; ++i ) {
         if( _current_frames[i].channels() == display_frame_.channels() ) {
           _current_frames[i].copyTo(display_frame_(roi[i]));
         }
         else {
           cv::cvtColor(_current_frames[i], display_frame_(roi[i]),
               cv::COLOR_GRAY2BGR);
         }
       }

       if( display_frame_.depth() != CV_8U ) {
         display_frame_.convertTo(display_frame_, CV_8U);
       }

       if ( !_current_disparity.empty() ) {

         cv::Mat disp;


         _current_disparity.convertTo(disp, CV_8U,
             255 / std::max(1., _stereo_matcher.currentMaxDisparity()));

         apply_colormap(disp, disp, COLORMAP_TURBO);

         const int r =
             _stereo_matcher.currentReferenceImageIndex();

         const cv::Rect &blend_roi =
             roi[2 + r];

         const cv::Rect &disp_roi =
             roi[2 + !r];

         disp.copyTo(display_frame_(disp_roi));

         if ( _current_frames[r].type() == display_frame_.type() ) {
           cv::addWeighted(disp, 0.5, _current_frames[r], 0.5, 0, display_frame_(blend_roi));
         }
         else if ( _current_frames[r].depth() == display_frame_.depth() ) {
           cv::Mat tmp;
           cv::cvtColor(_current_frames[r], tmp, cv::COLOR_GRAY2BGR);
           cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));
         }
         else if ( _current_frames[r].channels() == display_frame_.channels() ) {
           cv::Mat tmp;
           _current_frames[r].convertTo(tmp, display_frame_.depth());
           cv::addWeighted(disp, 0.5, tmp, 0.5, 0, display_frame_(blend_roi));

         }
         else {
           cv::Mat tmp;
           cv::cvtColor(_current_frames[r], tmp, cv::COLOR_GRAY2BGR);
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

