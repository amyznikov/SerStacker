/*
 * c_stereo_calibration_pipeline.cc
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#include "c_stereo_calibration_pipeline.h"
#include <core/settings/opencv_settings.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/io/load_image.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <chrono>
#include <thread>
#include <core/debug.h>


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
  return _input_options;
}

const c_stereo_calibration_input_options & c_stereo_calibration_pipeline::input_options() const
{
  return _input_options;
}

c_chessboard_corners_detection_options & c_stereo_calibration_pipeline::chessboard_detection_options()
{
  return chessboard_detection_options_;
}

const c_chessboard_corners_detection_options & c_stereo_calibration_pipeline::chessboard_detection_options() const
{
  return chessboard_detection_options_;
}

c_stereo_calibrate_options & c_stereo_calibration_pipeline::calibration_options()
{
  return calibration_options_;
}

const c_stereo_calibrate_options & c_stereo_calibration_pipeline::calibration_options() const
{
  return calibration_options_;
}

c_stereo_calibration_output_options & c_stereo_calibration_pipeline::output_options()
{
  return output_options_;
}

const c_stereo_calibration_output_options & c_stereo_calibration_pipeline::output_options() const
{
  return output_options_;
}

bool c_stereo_calibration_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_stereo_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "chessboard_detection")) ) {
    SERIALIZE_OBJECT(section, save, chessboard_detection_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "calibration_options")) ) {
    SERIALIZE_OPTION(section, save, calibration_options_, enable_calibration);
    SERIALIZE_OPTION(section, save, calibration_options_, min_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, max_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, calibration_flags);
    SERIALIZE_OPTION(section, save, calibration_options_, auto_tune_calibration_flags);
    SERIALIZE_OPTION(section, save, calibration_options_, init_camera_matrix_2d);
    SERIALIZE_OPTION(section, save, calibration_options_, max_iterations );
    SERIALIZE_OPTION(section, save, calibration_options_, solver_eps );
    SERIALIZE_OPTION(section, save, calibration_options_, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, output_intrinsics_filename);
    SERIALIZE_OPTION(section, save, output_options_, output_extrinsics_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_chessboard_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "chessboard_frames_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, chessboard_frames_output_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, save_rectified_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "rectified_frames_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, rectified_frames_output_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, save_stereo_rectified_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "stereo_rectified_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, stereo_rectified_output_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, save_quad_rectified_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "quad_rectified_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, quad_rectified_output_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "progress_video_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, progress_video_output_options);
    }
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl>& c_stereo_calibration_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_STEREO_INPUT_OPTIONS(ctrls)
      PIPELINE_CTL_GROUP(ctrls, "Input Sequence", "");
        POPULATE_PIPELINE_INPUT_OPTIONS(ctrls);
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Chessboard corners detection", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.method, "Method", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.chessboard_size, "chessboard_size", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.chessboard_cell_size, "chessboard_cell_size", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.chessboard_distance, "chessboard_distance", "distance to chessboard in [m]");


      PIPELINE_CTL_GROUP(ctrls, "findChessboardCorners", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_. findChessboardCorners.max_scales, "max_scales", "");
      PIPELINE_CTL_BITFLAGS(ctrls, chessboard_detection_options_.findChessboardCorners.flags, FindChessboardCornersFlags,"flags", "");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "findChessboardCornersSB", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.findChessboardCornersSB.max_scales, "max_scales", "");
      PIPELINE_CTL_BITFLAGS(ctrls, chessboard_detection_options_.findChessboardCornersSB.flags,FindChessboardCornersSBFlags, "flags", "");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "cornerSubPix options", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.cornerSubPix.winSize, "winSize", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.cornerSubPix.zeroZone, "zeroZone", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.cornerSubPix.max_solver_iterations, "max_solver_iterations", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.cornerSubPix.solver_eps, "solver_eps", "");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "BilateralFilter options", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.bilateralFilter.d, "d", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.bilateralFilter.sigmaColor, "sigmaColor", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.bilateralFilter.sigmaSpace, "sigmaSpace", "");
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_GROUP(ctrls, "Stereo Calibration", "");
      PIPELINE_CTL(ctrls, calibration_options_.enable_calibration, "enable calibration", ""); \
      PIPELINE_CTL(ctrls, calibration_options_.min_frames, "min_frames", "");
      PIPELINE_CTL(ctrls, calibration_options_.max_frames, "max_frames", "");
      PIPELINE_CTL_BITFLAGS(ctrls, calibration_options_.calibration_flags, STEREO_CALIBRATION_FLAGS,  "calibration flags", "" );
      PIPELINE_CTL(ctrls, calibration_options_.auto_tune_calibration_flags, "auto_tune_calibration_flags", "")
      PIPELINE_CTL(ctrls, calibration_options_.init_camera_matrix_2d, "init_camera_matrix_2d", "")
      PIPELINE_CTL(ctrls, calibration_options_.max_iterations, "max solver iterations", "")
      PIPELINE_CTL(ctrls, calibration_options_.solver_eps, "solver_eps", "")
      PIPELINE_CTL(ctrls, calibration_options_.filter_alpha, "corners filter alpha", "")
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL(ctrls, output_options_.output_intrinsics_filename, "output_intrinsics_filename", "");
      PIPELINE_CTL(ctrls, output_options_.output_extrinsics_filename, "output_extrinsics_filename", "");

      PIPELINE_CTL_GROUP(ctrls, "Save Chessboard Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_chessboard_frames, "save_chessboard_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.chessboard_frames_output_options,
            _this->output_options_.save_chessboard_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "Save Rectified Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_rectified_frames, "save_rectified_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.rectified_frames_output_options,
            _this->output_options_.save_rectified_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "Save Stereo Rectified Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_stereo_rectified_frames, "save_stereo_rectified_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.stereo_rectified_output_options,
            _this->output_options_.save_stereo_rectified_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "Save Quad Rectified Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_quad_rectified_frames, "save_quad_rectified_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.quad_rectified_output_options,
            _this->output_options_.save_quad_rectified_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "Save Progress Video", "");
        PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.progress_video_output_options,
            _this->output_options_.save_progress_video);
      PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}


bool c_stereo_calibration_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_image_stacking_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->chessboard_detection_options_ = this->chessboard_detection_options_;
  p->calibration_options_ = this->calibration_options_;
  p->output_options_ = this->output_options_;

  return true;

}

bool c_stereo_calibration_pipeline::read_stereo_frame()
{
  lock_guard lock(mutex());

  bool fOK =
      ::read_stereo_source(input_,
          _input_options.layout_type,
          _input_options.swap_cameras,
          _input_options.enable_color_maxtrix,
          current_frames_,
          current_masks_);

  if( !fOK ) {
    CF_ERROR("read_stereo_source() fails");
    return false;
  }

  for( int i = 0; i < 2; ++i ) {

    c_camera_intrinsics &intrinsics =
        current_intrinsics_.camera[i];

    if( intrinsics.image_size.empty() ) {
      intrinsics.image_size =
          current_frames_[0].size();
    }
    else if( intrinsics.image_size != current_frames_[0].size() ) {
      CF_ERROR("INPUT ERROR: Frame size change (%dx%d) -> (%dx%d) not supported\n",
          intrinsics.image_size.width, intrinsics.image_size.height,
          current_frames_[0].cols, current_frames_[0].rows);
      return false;
    }
  }

  if( _input_options.input_image_processor ) {
    for( int i = 0; i < 2; ++i ) {
      cv::Mat &image = current_frames_[i];
      cv::Mat &mask = current_masks_[i];

      if( !_input_options.input_image_processor->process(image, mask) ) {
        CF_ERROR("ERROR: input_image_processor->process() fails for stereo frame %d", i);
        return false;
      }
    }
  }

  return true;
}

bool c_stereo_calibration_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  _output_path =
      create_output_path(output_options_.output_directory);

  output_intrinsics_filename_ =
      generate_output_filename(output_options_.output_intrinsics_filename,
          "stereo_intrinsics",
          ".yml");

  output_extrinsics_filename_ =
      generate_output_filename(output_options_.output_extrinsics_filename,
          "stereo_extrinsics",
          ".yml");

  /////////////////////////////////////////////////////////////////////////////

  best_calibration_flags_ = current_calibration_flags_ = calibration_options_.calibration_flags;
  best_subset_quality_ = HUGE_VAL;
  stereo_intrinsics_initialized_ = false;

  if ( chessboard_detection_options_.chessboard_size.width < 2 || chessboard_detection_options_.chessboard_size.height < 2 ) {
    CF_ERROR("Invalid chessboard_detection_options_.chessboard_size: %dx%d", chessboard_detection_options_.chessboard_size.width,
        chessboard_detection_options_.chessboard_size.height);
    return false;
  }

  if ( !(chessboard_detection_options_.chessboard_cell_size.width > 0) || !(chessboard_detection_options_.chessboard_cell_size.height > 0)  ) {
    CF_ERROR("Invalid chessboard_cell_size_: %gx%g", chessboard_detection_options_.chessboard_cell_size.width,
        chessboard_detection_options_.chessboard_cell_size.height);
    return false;
  }

  current_object_points_.clear();
  current_object_points_.reserve(chessboard_detection_options_.chessboard_size.area());

  for( int i = 0; i < chessboard_detection_options_.chessboard_size.height; ++i ) {
    for( int j = 0; j < chessboard_detection_options_.chessboard_size.width; ++j ) {

      current_object_points_.emplace_back(
          j * chessboard_detection_options_.chessboard_cell_size.width,
          i * chessboard_detection_options_.chessboard_cell_size.height,
          chessboard_detection_options_.chessboard_distance);
    }
  }

  current_extrinsics_.R = cv::Matx33d::eye();
  current_extrinsics_.T = cv::Vec3d::all(0);
  best_extrinsics_.R = cv::Matx33d::eye();
  best_extrinsics_.T = cv::Vec3d::all(0);
  new_extrinsics_.R = cv::Matx33d::eye();
  new_extrinsics_.T = cv::Vec3d::all(0);

  for ( int i = 0; i < 2; ++i ) {
    current_intrinsics_.camera[i].image_size = cv::Size(0, 0);
    current_intrinsics_.camera[i].camera_matrix = cv::Matx33d::eye();
    current_intrinsics_.camera[i].dist_coeffs.clear();

    best_intrinsics_.camera[i].image_size = cv::Size(0, 0);
    best_intrinsics_.camera[i].camera_matrix = cv::Matx33d::eye();
    best_intrinsics_.camera[i].dist_coeffs.clear();

    new_intrinsics_.camera[i].image_size = cv::Size(0, 0);
    new_intrinsics_.camera[i].camera_matrix = cv::Matx33d::eye();
    new_intrinsics_.camera[i].dist_coeffs.clear();
  }

  /////////////////////////////////////////////////////////////////////////////

  const bool is_live_input =
      _input_sequence->is_live();

  if( !is_live_input ) {

    if( _input_options.left_stereo_source.empty() ) {
      CF_ERROR("ERROR: No left stereo source specified");
      return false;
    }

    if( _input_options.layout_type == stereo_frame_layout_separate_sources ) {
      if( _input_options.right_stereo_source.empty() ) {
        CF_ERROR("ERROR: No right stereo source specified");
        return false;
      }
    }

    input_.inputs[0] = _input_sequence->source(_input_options.left_stereo_source);
    if( !input_.inputs[0] ) {
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
  else if( _input_sequence->sources().empty() ) {
    CF_ERROR("ERROR: No stereo source specified");
    return false;
  }
  else {

    if( !(input_.inputs[0] = _input_sequence->source(0)) ) {
      CF_ERROR("ERROR: stereo source is null in input sequence");
      return false;
    }

    if( _input_options.layout_type == stereo_frame_layout_separate_sources ) {

      if( _input_sequence->sources().size() < 2 ) {
        CF_ERROR("ERROR: No second stereo source specified");
        return false;
      }

      if( !(input_.inputs[1] = _input_sequence->source(1)) ) {
        CF_ERROR("ERROR: second stereo source is null in input sequence");
        return false;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}


void c_stereo_calibration_pipeline::cleanup_pipeline()
{
  lock_guard lock(mutex());

  close_input_source();

  if( chessboard_video_writer_.is_open() ) {
    CF_DEBUG("closing '%s'", chessboard_video_writer_.filename().c_str());
    chessboard_video_writer_.close();
  }

  object_points_.clear();
  current_object_points_.clear();
  //  rvecs_.clear();
  //  tvecs_.clear();
  perViewErrors_.release();

  for ( int i = 0; i < 2; ++i ) {

    image_points_[i].clear();
    current_image_points_[i].clear();
    current_frames_[i].release();
    current_masks_[i].release();
    rmaps_[i].release();
  }

  base::cleanup_pipeline();
}


bool c_stereo_calibration_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if ( current_frames_[0].empty() || current_frames_[0].empty() ) {
    return false;
  }

  const cv::Size sizes[2] = {
      current_frames_[0].size(),
      current_frames_[1].size(),
  };

  const cv::Rect roi[4] = {
      cv::Rect(0, 0, sizes[0].width, sizes[0].height), // top left
      cv::Rect(sizes[0].width, 0, sizes[1].width, sizes[1].height), // top right
      cv::Rect(0, std::max(sizes[0].height, sizes[1].height), sizes[0].width, sizes[0].height), // bottom left
      cv::Rect(sizes[0].width, std::max(sizes[0].height, sizes[1].height), sizes[1].width, sizes[1].height), // bottom right
      };

  const cv::Size displaySize(
      sizes[0].width + sizes[1].width,
      2 * std::max(sizes[0].height, sizes[1].height));

  display_frame.create(displaySize,
      current_frames_[0].type());

  cv::Mat & display_frame_ =
      display_frame.getMatRef();

  current_frames_[0].copyTo(display_frame_(roi[0]));
  current_frames_[1].copyTo(display_frame_(roi[1]));

  if( rmaps_[0].empty() ) {
    current_frames_[0].copyTo(display_frame_(roi[2]));
    current_frames_[1].copyTo(display_frame_(roi[3]));
  }

  else {

    cv::remap(current_frames_[0], display_frame_(roi[2]),
        rmaps_[0], cv::noArray(),
        cv::INTER_LINEAR);

    cv::remap(current_frames_[1], display_frame_(roi[3]),
        rmaps_[1], cv::noArray(),
        cv::INTER_LINEAR);
  }

  if( !current_image_points_[0].empty() ) {

    if ( display_frame_.channels() == 1 ) {
      cv::cvtColor(display_frame_, display_frame_,
          cv::COLOR_GRAY2BGR);
    }

    cv::drawChessboardCorners(display_frame_(roi[0]),
        chessboard_detection_options_.chessboard_size,
        current_image_points_[0],
        true);
  }

  if( !current_image_points_[1].empty() ) {

    if ( display_frame_.channels() == 1 ) {
      cv::cvtColor(display_frame_, display_frame_,
          cv::COLOR_GRAY2BGR);
    }

    cv::drawChessboardCorners(display_frame_(roi[1]),
        chessboard_detection_options_.chessboard_size,
        current_image_points_[1],
        true);
  }

  if( rmaps_[0].empty() ) {

    // draw coverage

    for ( int i = 0; i < 2; ++i ) {

      cv::Mat3b display =
          display_frame_(roi[i + 2]);

      for( const std::vector<cv::Point2f> &corners : image_points_[i] ) {
        for( const cv::Point2f &corner : corners ) {
          cv::rectangle(display, cv::Rect(corner.x - 1, corner.y - 1, 3, 3), CV_RGB(0, 255, 0), -1, cv::LINE_8);
        }
      }
    }
  }


  if( display_frame.needed() ) {
    display_frame_.copyTo(display_frame);
  }

  if( display_mask.needed() ) {
    //display_mask_.copyTo(display_mask);
    display_mask.release();
  }

  return true;
}

void c_stereo_calibration_pipeline::close_input_source()
{
  ::close_stereo_source(input_);
}

bool c_stereo_calibration_pipeline::open_input_source()
{
//  if ( !input_sources_[0]->open() ) {
//    CF_ERROR("ERROR: can not open input source '%s'", input_sources_[0]->cfilename());
//    return false;
//  }
//
//  if ( _input_options.layout_type == stereo_frame_layout_separate_sources ) {
//    if ( !input_sources_[1]->open() ) {
//      CF_ERROR("ERROR: can not open input source '%s'", input_sources_[1]->cfilename());
//      return false;
//    }
//
//    if( input_sources_[0]->size() != input_sources_[1]->size() ) {
//      CF_ERROR("ERROR: input sources sizes not match: left size=%d right size=%d ",
//          input_sources_[0]->size(), input_sources_[1]->size());
//      return false;
//    }
//  }

  return ::open_stereo_source(input_, _input_options.layout_type);
}

bool c_stereo_calibration_pipeline::seek_input_source(int pos)
{
//  if( !input_sources_[0]->seek(pos) ) {
//    CF_ERROR("ERROR: input_sources_[0]->seek(pos=%d) fails", pos);
//    return false;
//  }
//
//  if( _input_options.layout_type == stereo_frame_layout_separate_sources ) {
//    if( !input_sources_[1]->seek(pos) ) {
//      CF_ERROR("ERROR: input_sources_[1]->seek(pos=%d) fails", pos);
//      return false;
//    }
//  }

  return ::seek_stereo_source(input_, pos);
}


bool c_stereo_calibration_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  ////////////

  if ( _input_sequence->is_live() ) {
    _total_frames = INT_MAX;
    _processed_frames = 0;
    _accumulated_frames = 0;
  }
  else {

    if ( !open_input_source() ) {
      CF_ERROR("ERROR: open_input_source() fails");
      return false;
    }

    const int start_pos =
        std::max(_input_options.start_frame_index, 0);

    const int end_pos =
        _input_options.max_input_frames < 1 ?
            input_.inputs[0]->size() :
            std::min(input_.inputs[0]->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);


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

  const bool enable_live_calibration =
      calibration_options_.enable_calibration &&
      _input_sequence->is_live();

  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

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

    if ( !process_current_stereo_frame(enable_live_calibration) ) {
      CF_ERROR("process_current_stereo_frame(fails)");
      break;
    }

    _accumulated_frames =
        object_points_.size();

    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  close_input_source();

  if ( calibration_options_.enable_calibration && !_input_sequence->is_live()  ) {

    if ( !update_calibration() ) {
      CF_ERROR("update_stereo_calibration() fails");
      return false;
    }

    if ( !write_output_videos() ) {
      return false;
    }
  }

  return true;
}

bool c_stereo_calibration_pipeline::update_calibration()
{
  if( !stereo_intrinsics_initialized_ && calibration_options_.init_camera_matrix_2d ) {

    const cv::Size image_size =
        current_frames_[0].size();

    stereo_intrinsics_initialized_ =
        init_camera_intrinsics(current_intrinsics_,
            object_points_,
            image_points_[0],
            image_points_[1],
            image_size,
            1);

    if( !stereo_intrinsics_initialized_ ) {
      CF_ERROR("init_camera_intrinsics() fails");
      return false; // wait for next frame
    }

    const cv::Matx33d &M0 =
        current_intrinsics_.camera[0].camera_matrix;

    const cv::Matx33d &M1 =
        current_intrinsics_.camera[1].camera_matrix;

    CF_DEBUG("\nINITIAL M0: {\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "}\n"

        "\nINITIAL M1: {\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "}\n",

    M0(0, 0), M0(0, 1), M0(0, 2),
        M0(1, 0), M0(1, 1), M0(1, 2),
        M0(2, 0), M0(2, 1), M0(2, 2),

        M1(0, 0), M1(0, 1), M1(0, 2),
        M1(1, 0), M1(1, 1), M1(1, 2),
        M1(2, 0), M1(2, 1), M1(2, 2));
  }

  if( best_subset_quality_ < HUGE_VAL ) {
    current_intrinsics_ = best_intrinsics_;
    current_extrinsics_ = best_extrinsics_;
    current_calibration_flags_ = best_calibration_flags_;
  }

  rmse_ =
      stereo_calibrate(object_points_,
          image_points_[0], image_points_[1],
          current_intrinsics_,
          current_extrinsics_,
          current_calibration_flags_,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
              calibration_options_.max_iterations,
              calibration_options_.solver_eps),
          &E_,
          &F_,
          // &rvecs_,
          // &tvecs_,
          &perViewErrors_);

  const cv::Matx33d &M0 =
      current_intrinsics_.camera[0].camera_matrix;

  const cv::Matx33d &M1 =
      current_intrinsics_.camera[1].camera_matrix;

  CF_DEBUG("\nM0: {\n"
      "  %+g %+g %+g\n"
      "  %+g %+g %+g\n"
      "  %+g %+g %+g\n"
      "}\n"

      "\nM1: {\n"
      "  %+g %+g %+g\n"
      "  %+g %+g %+g\n"
      "  %+g %+g %+g\n"
      "}\n"
      "\n"
      "RMSE=%g\n",

      M0(0, 0), M0(0, 1), M0(0, 2),
      M0(1, 0), M0(1, 1), M0(1, 2),
      M0(2, 0), M0(2, 1), M0(2, 2),

      M1(0, 0), M1(0, 1), M1(0, 2),
      M1(1, 0), M1(1, 1), M1(1, 2),
      M1(2, 0), M1(2, 1), M1(2, 2),
      rmse_);

  if( rmse_ >= 0 ) {

    const double subset_quality =
        estimate_subset_quality();

    stereo_intrinsics_initialized_ = true;

    CF_DEBUG("subset_quality=%g best_subset_quality_=%g",
        subset_quality, best_subset_quality_);

    if( subset_quality < best_subset_quality_ ) {

      best_intrinsics_ = current_intrinsics_;
      best_extrinsics_ = current_extrinsics_;
      best_calibration_flags_ = current_calibration_flags_;
      best_subset_quality_ = subset_quality;

      update_state();

      if( !save_current_camera_parameters() ) {
        CF_ERROR("save_current_camera_parameters() fails");
        return false;
      }

      update_undistortion_remap();

      return true;
    }
  }


  return false;
}


double c_stereo_calibration_pipeline::estimate_subset_quality() const
{
  const int nbframes =
      object_points_.size();

  if( nbframes > 0 ) {

    double grid_mean, grid_stdev;

    estimate_grid_meanstdev(&grid_mean, &grid_stdev, nbframes);

    const double grid_quality =
        grid_stdev / grid_mean;


    double rmse_quality = 0;
    for( size_t i = 0; i < nbframes; i++ ) {
      rmse_quality += (perViewErrors_[i][0] + perViewErrors_[i][1]);
    }
    rmse_quality /= nbframes;


    const double alpha =
        calibration_options_.filter_alpha;

    CF_DEBUG("grid: mean=%g stdev=%g quality=%g ; rmse: %g",
        grid_mean, grid_stdev, grid_quality, rmse_quality);

    return 0.5 * rmse_quality * alpha + grid_quality * (1 - alpha);
  }

  return 0;
}

void c_stereo_calibration_pipeline::estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const
{
  const cv::Size image_size = current_frames_[0].size();

  const int gridSize = 10;
  const int xGridStep = image_size.width / gridSize;
  const int yGridStep = image_size.height / gridSize;
  const int stride = gridSize * gridSize;

  std::vector<int> pointsInCell(2 * stride);

  std::fill(pointsInCell.begin(), pointsInCell.end(), 0);

  for( int k = 0; k < object_points_.size(); k++ )
    if( k != excludedIndex ) {

      for( int i = 0; i < 2; ++i ) {

        for( const auto &p : image_points_[i][k] ) {

          int ii = (int) (p.x / xGridStep);
          int jj = (int) (p.y / yGridStep);

          pointsInCell[ii * gridSize + jj]++;
          pointsInCell[ii * gridSize + jj + stride]++;
        }
      }

    }

  cv::Scalar mean, stdDev;
  cv::meanStdDev(pointsInCell, mean, stdDev);

  if( m ) {
    *m = mean[0];
  }
  if( s ) {
    *s = stdDev[0];
  }
}

double c_stereo_calibration_pipeline::estimate_coverage_quality(int excludedIndex) const
{
  double mean, stdev;

  estimate_grid_meanstdev(&mean, &stdev,
      excludedIndex);

  return stdev / mean;
}

void c_stereo_calibration_pipeline::filter_frames(bool only_landmarks)
{
  const int nbframes =
      object_points_.size();

  const int nbframesmax =
      std::max(1, std::max(calibration_options_.min_frames,
          calibration_options_.max_frames));

  //CF_DEBUG("nbframes = %d / %d", nbframes, nbframesmax);

  if( !only_landmarks && nbframes != perViewErrors_.rows ) {

    if( nbframes > 1 ) {
      CF_ERROR("APP BUG: object_points_.size()=%zu != current_per_view_errors_.total()=%d",
          object_points_.size(),
          perViewErrors_.rows);
    }

    if( nbframes > nbframesmax ) {
      image_points_[0].erase(image_points_[0].begin());
      image_points_[1].erase(image_points_[1].begin());
      object_points_.erase(object_points_.begin());
    }

    return;
  }


  if ( nbframes > nbframesmax ) {

    static const auto estimateRmeQuality =
        [](const cv::Mat1d & perViewErrors) -> double {
          double sum = 0;
          for( int i = 0, n = perViewErrors.rows; i < n; ++i ) {
            sum += (perViewErrors[i][0] + perViewErrors[i][1]);
          }
          return 0.5 * sum;
        };


    const double alpha =
        calibration_options_.filter_alpha;

    const double totalRmeQuality =
        only_landmarks ? 0 :
            estimateRmeQuality(perViewErrors_);

    const double totalCoverageQuality =
        estimate_coverage_quality(nbframes);

    double bestSubsetQuality = HUGE_VAL;
    int worstElemIndex = 0;

    for( int i = 0; i < nbframes; ++i ) {

      const double currentCoverageQuality =
          estimate_coverage_quality(i);

      double currentSubsetQuality;

      if ( only_landmarks ) {
        currentSubsetQuality =
            currentCoverageQuality;
      }
      else {

        const double currentRmseQuality =
            (totalRmeQuality - 0.5 * (perViewErrors_[i][0] + perViewErrors_[i][1])) / (perViewErrors_.rows - 1);

        currentSubsetQuality =
            alpha * currentRmseQuality + (1. - alpha) * currentCoverageQuality;
      }

      if( currentSubsetQuality < bestSubsetQuality ) {
        bestSubsetQuality = currentSubsetQuality;
        worstElemIndex = i;
      }
    }

    // CF_DEBUG("worstElemIndex=%d bestSubsetQuality=%g", worstElemIndex, bestSubsetQuality);

    image_points_[0].erase(image_points_[0].begin() + worstElemIndex);
    image_points_[1].erase(image_points_[1].begin() + worstElemIndex);
    object_points_.erase(object_points_.begin() + worstElemIndex);

    if ( nbframes == perViewErrors_.rows ) {

      cv::Mat1f newErrorsVec(nbframes - 1,
          perViewErrors_.cols);

      std::copy(perViewErrors_[0],
          perViewErrors_[worstElemIndex],
          newErrorsVec[0]);

      if( worstElemIndex < nbframes - 1 ) {
        std::copy(perViewErrors_[worstElemIndex + 1],
            perViewErrors_[nbframes],
            newErrorsVec[worstElemIndex]);
      }

      perViewErrors_ = newErrorsVec;
    }
  }
}

void c_stereo_calibration_pipeline::update_state()
{
  if( calibration_options_.auto_tune_calibration_flags && object_points_.size() > calibration_options_.min_frames ) {

    if( !(current_calibration_flags_ & cv::CALIB_ZERO_TANGENT_DIST) ) {

      const double eps = 0.005;

      bool fix_zero_tangent_dist = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            current_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 3) && (fabs(D[2]) > eps || fabs(D[3]) > eps) ) {
          fix_zero_tangent_dist = false;
          break;
        }
      }

      if( fix_zero_tangent_dist ) {
        current_calibration_flags_ |= cv::CALIB_ZERO_TANGENT_DIST;
      }
    }

    if( !(current_calibration_flags_ & cv::CALIB_FIX_K1) ) {

      const double eps = 0.005;

      bool fix_k1 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            current_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 0) && (fabs(D[0]) > eps) ) {
          fix_k1 = false;
          break;
        }
      }

      if( fix_k1 ) {
        current_calibration_flags_ |= cv::CALIB_FIX_K1;
      }
    }

    if( !(current_calibration_flags_ & cv::CALIB_FIX_K2) ) {

      const double eps = 0.005;

      bool fix_k2 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            current_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 1) && (fabs(D[1]) > eps) ) {
          fix_k2 = false;
          break;
        }
      }

      if( fix_k2 ) {
        current_calibration_flags_ |= cv::CALIB_FIX_K2;
      }
    }

    if( !(current_calibration_flags_ & cv::CALIB_FIX_K3) ) {

      const double eps = 0.005;

      bool fix_k3 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            current_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 4) && (fabs(D[4]) > eps) ) {
          fix_k3 = false;
          break;
        }
      }

      if( fix_k3 ) {
        current_calibration_flags_ |= cv::CALIB_FIX_K3;
      }
    }
  }
}


void c_stereo_calibration_pipeline::update_undistortion_remap()
{
  const cv::Size & image_size =
      best_intrinsics_.camera[0].image_size;

  create_stereo_rectification(image_size,
      best_intrinsics_,
      best_extrinsics_,
      -1,
      rmaps_,
      &new_intrinsics_,
      &new_extrinsics_,
      nullptr,
      nullptr,
      nullptr,
      nullptr);
}

bool c_stereo_calibration_pipeline::save_current_camera_parameters() const
{
  if( !output_intrinsics_filename_.empty() ) {

    if( !create_path(get_parent_directory(output_intrinsics_filename_)) ) {
      CF_ERROR("create_path('%s') fails: %s", output_intrinsics_filename_.c_str(), strerror(errno));
      return false;
    }

    CF_DEBUG("saving output_intrinsics_filename_: %s", output_intrinsics_filename_.c_str());

    if( !write_stereo_camera_intrinsics_yml(best_intrinsics_, output_intrinsics_filename_) ) {
      CF_ERROR("ERROR: save_stereo_camera_intrinsics_yml('%s') fails",
          output_intrinsics_filename_.c_str());
      return false;
    }
  }

  if( !output_extrinsics_filename_.empty() ) {

    if( !create_path(get_parent_directory(output_extrinsics_filename_)) ) {
      CF_ERROR("create_path('%s') fails: %s", output_extrinsics_filename_.c_str(), strerror(errno));
      return false;
    }

    CF_DEBUG("saving output_extrinsics_filename_: %s", output_extrinsics_filename_.c_str());

    if( !write_stereo_camera_extrinsics_yml(best_extrinsics_, output_extrinsics_filename_) ) {
      CF_ERROR("ERROR: save_stereo_camera_extrinsics_yml('%s') fails",
          output_extrinsics_filename_.c_str());
    }
  }

  return true;
}

//bool c_stereo_calibration_pipeline::run_chessboard_corners_collection()
//{
//  CF_DEBUG("Starting '%s: %s' ...",
//      csequence_name(), cname());
//
//
//  if ( !open_input_source() ) {
//    CF_ERROR("ERROR: open_input_source() fails");
//    return false;
//  }
//
//  const int start_pos =
//      std::max(_input_options.start_frame_index, 0);
//
//  const int end_pos =
//      _input_options.max_input_frames < 1 ?
//          input_.inputs[0]->size() :
//          std::min(input_.inputs[0]->size(),
//              _input_options.start_frame_index + _input_options.max_input_frames);
//
//
//  total_frames_ = end_pos - start_pos;
//  processed_frames_ = 0;
//  accumulated_frames_ = 0;
//
//  if( total_frames_ < 1 ) {
//    CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
//        total_frames_);
//    return false;
//  }
//
//  if( !seek_input_source(start_pos) ) {
//    CF_ERROR("ERROR: seek_input_source(start_pos=%d) fails", start_pos);
//    return false;
//  }
//
//  set_status_msg("RUNNING ...");
//
//  bool fOK = true;
//
//  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {
//
//    fOK = true;
//
//    if ( canceled() ) {
//      break;
//    }
//
//    if ( !read_stereo_frame() ) {
//      CF_ERROR("read_stereo_frame() fails");
//      break;
//    }
//
//    if ( canceled() ) {
//      break;
//    }
//
//    if ( !process_current_stereo_frame(false) ) {
//      break;
//    }
//
//    accumulated_frames_ =
//        object_points_.size();
//
//
//    // give chance to GUI thread to call get_display_image()
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//  }
//
//  close_input_source();
//
//  return !canceled();
//}


bool c_stereo_calibration_pipeline::detect_chessboard(const cv::Mat & frame, std::vector<cv::Point2f> & corners_) const
{
  return find_chessboard_corners(frame,
      chessboard_detection_options_.chessboard_size,
      corners_,
      chessboard_detection_options_);
}

bool c_stereo_calibration_pipeline::process_current_stereo_frame(bool enable_calibration)
{

  if ( true ) {

    lock_guard lock(mutex());

    for( int i = 0; i < 2; ++i ) {
      current_image_points_[i].clear();
    }

    for( int i = 0; i < 2; ++i ) {
      if( !detect_chessboard(current_frames_[i], current_image_points_[i]) ) {
        return true; // wait for next frame
      }
    }
  }

  if ( canceled() ) {
    return false;
  }

  if ( !write_chessboard_video() ) {
    CF_ERROR("write_frames_with_detected_chessboard() fails");
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if ( true ) {

    lock_guard lock(mutex());

    image_points_[0].emplace_back(current_image_points_[0]);
    image_points_[1].emplace_back(current_image_points_[1]);
    object_points_.emplace_back(current_object_points_);

    if( object_points_.size() >= std::max(1, calibration_options_.min_frames) ) {

      if( !enable_calibration ) {

        filter_frames(true);

      }
      else if( update_calibration() ) {

        if ( canceled() ) {
          return false;
        }

        filter_frames(false);
      }
    }
  }

  return true;
}

bool c_stereo_calibration_pipeline::write_chessboard_video()
{
  if( !output_options_.save_chessboard_frames ) {
    return true;
  }

  const cv::Size sizes[2] = {
      current_frames_[0].size(),
      current_frames_[1].size(),
  };

  const cv::Size totalsize(sizes[0].width + sizes[1].width,
      std::max(sizes[0].height, sizes[1].height));

  const cv::Rect roi[2] = {
      cv::Rect(0, 0, sizes[0].width, sizes[0].height),
      cv::Rect(sizes[0].width, 0, sizes[1].width, sizes[1].height),
  };

  cv::Mat frame(totalsize, current_frames_[0].type());
  current_frames_[0].copyTo(frame(roi[0]));
  current_frames_[1].copyTo(frame(roi[1]));

  if( !chessboard_video_writer_.is_open() ) {

    bool fOK =
        open_output_writer(chessboard_video_writer_,
            output_options_.chessboard_frames_output_options,
            "chessboard",
            ".avi");

    if( !fOK ) {
      CF_ERROR("chessboard_video_writer_.open('%s') fails",
          chessboard_video_writer_.filename().c_str());
      return false;
    }

    CF_DEBUG("Created '%s'", chessboard_video_writer_.filename().c_str());
  }

  if( !chessboard_video_writer_.write(frame, cv::noArray(), false, _input_sequence->current_pos() - 1) ) {
    CF_ERROR("chessboard_video_writer_.write() fails");
    return false;
  }

  return true;
}

bool c_stereo_calibration_pipeline::write_output_videos()
{
  if( !output_options_.save_rectified_frames &&
      !output_options_.save_stereo_rectified_frames &&
      !output_options_.save_quad_rectified_frames ) {
    return true;
  }

  CF_DEBUG("update_undistortion_remap()...");
  update_undistortion_remap();

  CF_DEBUG("Save rectified videos...");

  c_output_frame_writer video_writer[2];
  c_output_frame_writer stereo_writer;
  c_output_frame_writer quad_writer;

  cv::Size sizes[2];
  cv::Size stereo_size;
  cv::Size quad_size;
  cv::Mat remapped_frames[2];
  cv::Mat display_frame;

  if ( !open_input_source() ) {
    CF_ERROR("ERROR: open_input_source() fails");
    return false;
  }

  CF_DEBUG("Saving debug videos...");

  _total_frames = input_.inputs[0]->size();
  _processed_frames = 0;
  _accumulated_frames = 0;

  bool fOK;

  for( ; _processed_frames < _total_frames; ++_processed_frames,  ++_accumulated_frames, on_frame_processed() ) {

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

    for( int i = 0; i < 2; ++i ) {
      cv::remap(current_frames_[i], remapped_frames[i],
          rmaps_[i], cv::noArray(),
          cv::INTER_LINEAR);
    }

    sizes[0] = remapped_frames[0].size();
    sizes[1] = remapped_frames[1].size();

    if( output_options_.save_rectified_frames ) {

      for( int i = 0; i < 2; ++i ) {

        if( !video_writer[i].is_open() ) {

          fOK =
              open_output_writer(video_writer[i],
                  output_options_.rectified_frames_output_options,
                  i == 0 ? "rect-left" : "rect-right",
                  ".avi");

          if( !fOK ) {
            CF_ERROR("video_writer[%d].open() fails", i);
            return false;
          }
        }

        if( !video_writer[i].write(remapped_frames[0], cv::noArray(), false, _processed_frames) ) {
          CF_ERROR("video_writer[%d].write() fails", i);
          return false;
        }
      }
    }

    if( output_options_.save_stereo_rectified_frames ) {

      if( !stereo_writer.is_open() ) {

        stereo_size.width = sizes[0].width + sizes[1].width;
        stereo_size.height = std::max(sizes[0].height, sizes[1].height);

        fOK =
            open_output_writer(stereo_writer,
                output_options_.stereo_rectified_output_options,
                "stereo",
                ".avi");

        if( !fOK ) {
          CF_ERROR("stereo_writer.open() fails");
          return false;
        }
      }

      const cv::Rect rc0(0, 0, sizes[0].width, sizes[0].height);
      const cv::Rect rc1(sizes[0].width, 0, sizes[1].width, sizes[1].height);

      display_frame.create(stereo_size, remapped_frames[0].type());
      remapped_frames[0].copyTo(display_frame(rc0));
      remapped_frames[1].copyTo(display_frame(rc1));

      if( !stereo_writer.write(display_frame, cv::noArray(), false, _processed_frames) ) {
        CF_ERROR("stereo_writer.write() fails");
        return false;
      }
    }

    if ( output_options_.save_quad_rectified_frames ) {

      if( !quad_writer.is_open() ) {

        quad_size.width = sizes[0].width + sizes[1].width;
        quad_size.height = sizes[0].height + sizes[1].height;

        fOK =
            open_output_writer(quad_writer,
                output_options_.quad_rectified_output_options,
                "quad",
                ".avi");

        if( !fOK ) {
          CF_ERROR("quad_writer.open() fails");
          return false;
        }
      }

      const cv::Rect rc0(0, 0, sizes[0].width, sizes[0].height); // tl -> 0
      const cv::Rect rc1(sizes[0].width, 0, sizes[1].width, sizes[1].height); // tr -> 1
      const cv::Rect rc2(0, sizes[0].height, sizes[1].width, sizes[1].height); // bl -> 1
      const cv::Rect rc3(sizes[0].width, sizes[0].height, sizes[1].width, sizes[1].height); // bl -> blend

      display_frame.create(quad_size, remapped_frames[0].type());
      remapped_frames[0].copyTo(display_frame(rc0));
      remapped_frames[1].copyTo(display_frame(rc1));
      remapped_frames[1].copyTo(display_frame(rc2));
      cv::addWeighted(remapped_frames[0], 0.5, remapped_frames[1], 0.5, 0, display_frame(rc3));

      if ( !quad_writer.write(display_frame, cv::noArray(), false, _processed_frames ) ) {
        CF_ERROR("quad_writer.write() fails");
        return false;
      }
    }

    if( canceled() ) {
      break;
    }

    // give change to GUI thread calling get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }


  return true;
}


