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
  _input_options.input_source.input_sequence = input_sequence.get();
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
  return _chessboard_detection_options;
}

const c_chessboard_corners_detection_options & c_stereo_calibration_pipeline::chessboard_detection_options() const
{
  return _chessboard_detection_options;
}

c_stereo_calibrate_options & c_stereo_calibration_pipeline::calibration_options()
{
  return _calibration_options;
}

const c_stereo_calibrate_options & c_stereo_calibration_pipeline::calibration_options() const
{
  return _calibration_options;
}

c_stereo_calibration_output_options & c_stereo_calibration_pipeline::output_options()
{
  return _output_options;
}

const c_stereo_calibration_output_options & c_stereo_calibration_pipeline::output_options() const
{
  return _output_options;
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
    SERIALIZE_OBJECT(section, save, _chessboard_detection_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "calibration_options")) ) {
    SERIALIZE_OPTION(section, save, _calibration_options, enable_calibration);
    SERIALIZE_OPTION(section, save, _calibration_options, min_frames);
    SERIALIZE_OPTION(section, save, _calibration_options, max_frames);
    SERIALIZE_OPTION(section, save, _calibration_options, calibration_flags);
    SERIALIZE_OPTION(section, save, _calibration_options, auto_tune_calibration_flags);
    SERIALIZE_OPTION(section, save, _calibration_options, init_camera_matrix_2d);
    SERIALIZE_OPTION(section, save, _calibration_options, max_iterations );
    SERIALIZE_OPTION(section, save, _calibration_options, solver_eps );
    SERIALIZE_OPTION(section, save, _calibration_options, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);

    SERIALIZE_OPTION(section, save, _output_options, output_intrinsics_filename);
    SERIALIZE_OPTION(section, save, _output_options, output_extrinsics_filename);

    SERIALIZE_OPTION(section, save, _output_options, save_chessboard_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "chessboard_frames_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, chessboard_frames_output_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_rectified_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "rectified_frames_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, rectified_frames_output_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_stereo_rectified_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "stereo_rectified_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, stereo_rectified_output_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_quad_output_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "quad_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, quad_output_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_quad_rectified_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "quad_rectified_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, quad_rectified_output_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "progress_video_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, progress_video_output_options);
    }
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_calibration_input_options> & ctx)
{
  using S = c_stereo_calibration_input_options;
  ctlbind(ctls, as_base<c_stereo_input_options>(ctx));
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_calibrate_options> & ctx)
{
  using S = c_stereo_calibrate_options;

  ctlbind(ctls, "enable calibration",ctx(&S::enable_calibration),  "");
  ctlbind(ctls, "min_frames", ctx(&S::min_frames), "");
  ctlbind(ctls, "max_frames", ctx(&S::max_frames), "");

  ctlbind_flags_checkbox<STEREO_CALIBRATION_FLAGS>(ctls,"calibration flags", ctx(&S::calibration_flags), "" );

  ctlbind(ctls, "auto_tune_calibration_flags", ctx(&S::auto_tune_calibration_flags), "");
  ctlbind(ctls, "init_camera_matrix_2d", ctx(&S::init_camera_matrix_2d), "");
  ctlbind(ctls, "max solver iterations", ctx(&S::max_iterations), "");
  ctlbind(ctls, "solver_eps",ctx(&S::solver_eps),  "");
  ctlbind(ctls, "corners filter alpha", ctx(&S::filter_alpha), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_stereo_calibration_output_options> & ctx)
{
  using S = c_stereo_calibration_output_options;

  ctlbind_browse_for_file(ctls, "output_intrinsics_filename", ctx(&S::output_intrinsics_filename), "");
  ctlbind_browse_for_file(ctls, "output_extrinsics_filename", ctx(&S::output_extrinsics_filename), "");

  ctlbind_expandable_group(ctls, "Save Chessboard Frames", "");
  ctlbind(ctls, "save_chessboard_frames", ctx(&S::save_chessboard_frames), "");
    ctlbind(ctls, ctx(&S::chessboard_frames_output_options)); // _this->_output_options.save_chessboard_frames);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save Rectified Frames", "");
  ctlbind(ctls, "save_rectified_frames", ctx(&S::save_rectified_frames), "");
    ctlbind(ctls, ctx(&S::rectified_frames_output_options)); // _this->_output_options.save_rectified_frames);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save Stereo Rectified Frames", "");
  ctlbind(ctls, "save_stereo_rectified_frames", ctx(&S::save_stereo_rectified_frames), "");
    ctlbind(ctls, ctx(&S::stereo_rectified_output_options));//, _this->_output_options.save_stereo_rectified_frames);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save Quad Frames", "");
  ctlbind(ctls, "save_quad_output_options", ctx(&S::save_quad_output_frames), "");
    ctlbind(ctls, ctx(&S::quad_output_options));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save Quad Rectified Frames", "");
  ctlbind(ctls, "save_quad_rectified_frames", ctx(&S::save_quad_rectified_frames), "");
    ctlbind(ctls, ctx(&S::quad_rectified_output_options));// _this->_output_options.save_quad_rectified_frames);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save Progress Video", "");
  ctlbind(ctls, "save_progress_video", ctx(&S::save_progress_video), "");
    ctlbind(ctls, ctx(&S::progress_video_output_options)); // _this->_output_options.save_progress_video);
  ctlbind_end_group(ctls);

}

const c_ctlist<c_stereo_calibration_pipeline> & c_stereo_calibration_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "1. Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "2. Chessboard corners detection", "");
      ctlbind(ctls, ctx(&this_class::_chessboard_detection_options));

      ctlbind_menu_button(ctls, "Options...", ctx);
        ctlbind_command_button(ctls, "Copy config to clipboard", ctx,
            std::function([](this_class * _ths) {
              ctlbind_copy_config_to_clipboard("c_chessboard_corners_detection_options", _ths->_chessboard_detection_options);
              return false;
            }));
        ctlbind_command_button(ctls, "Paste config from clipboard", ctx,
            std::function([](this_class * _ths) {
              return  ctlbind_paste_config_from_clipboard("c_chessboard_corners_detection_options", &_ths->_chessboard_detection_options);
            }));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "3. Stereo Calibration", "");
      ctlbind(ctls, ctx(&this_class::_calibration_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "4. Output options", "");
      ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}

//const std::vector<c_image_processing_pipeline_ctrl>& c_stereo_calibration_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
////
////  if( ctrls.empty() ) {
////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////      POPULATE_PIPELINE_STEREO_INPUT_OPTIONS(ctrls)
////      PIPELINE_CTL_GROUP(ctrls, "Input Sequence", "");
////        POPULATE_PIPELINE_INPUT_OPTIONS(ctrls);
////      PIPELINE_CTL_END_GROUP(ctrls);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Chessboard corners detection", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.method, "Method", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.chessboard_size, "chessboard_size", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.chessboard_cell_size, "chessboard_cell_size", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.chessboard_distance, "chessboard_distance", "distance to chessboard in [m]");
////
////
////      PIPELINE_CTL_GROUP(ctrls, "findChessboardCorners", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options. findChessboardCorners.max_scales, "max_scales", "");
////      PIPELINE_CTL_BITFLAGS(ctrls, _chessboard_detection_options.findChessboardCorners.flags, FindChessboardCornersFlags,"flags", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "findChessboardCornersSB", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.findChessboardCornersSB.max_scales, "max_scales", "");
////      PIPELINE_CTL_BITFLAGS(ctrls, _chessboard_detection_options.findChessboardCornersSB.flags,FindChessboardCornersSBFlags, "flags", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "cornerSubPix options", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.winSize, "winSize", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.zeroZone, "zeroZone", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.max_solver_iterations, "max_solver_iterations", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.solver_eps, "solver_eps", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "BilateralFilter options", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.bilateralFilter.d, "d", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.bilateralFilter.sigmaColor, "sigmaColor", "");
////      PIPELINE_CTL(ctrls, _chessboard_detection_options.bilateralFilter.sigmaSpace, "sigmaSpace", "");
////      PIPELINE_CTL_END_GROUP(ctrls);
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////
////    PIPELINE_CTL_GROUP(ctrls, "Stereo Calibration", "");
////      PIPELINE_CTL(ctrls, _calibration_options.enable_calibration, "enable calibration", ""); \
////      PIPELINE_CTL(ctrls, _calibration_options.min_frames, "min_frames", "");
////      PIPELINE_CTL(ctrls, _calibration_options.max_frames, "max_frames", "");
////      PIPELINE_CTL_BITFLAGS(ctrls, _calibration_options.calibration_flags, STEREO_CALIBRATION_FLAGS,  "calibration flags", "" );
////      PIPELINE_CTL(ctrls, _calibration_options.auto_tune_calibration_flags, "auto_tune_calibration_flags", "")
////      PIPELINE_CTL(ctrls, _calibration_options.init_camera_matrix_2d, "init_camera_matrix_2d", "")
////      PIPELINE_CTL(ctrls, _calibration_options.max_iterations, "max solver iterations", "")
////      PIPELINE_CTL(ctrls, _calibration_options.solver_eps, "solver_eps", "")
////      PIPELINE_CTL(ctrls, _calibration_options.filter_alpha, "corners filter alpha", "")
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////      PIPELINE_CTL(ctrls, _output_options.output_intrinsics_filename, "output_intrinsics_filename", "");
////      PIPELINE_CTL(ctrls, _output_options.output_extrinsics_filename, "output_extrinsics_filename", "");
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Chessboard Frames", "");
////        PIPELINE_CTL(ctrls, _output_options.save_chessboard_frames, "save_chessboard_frames", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.chessboard_frames_output_options,
////            _this->_output_options.save_chessboard_frames);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Rectified Frames", "");
////        PIPELINE_CTL(ctrls, _output_options.save_rectified_frames, "save_rectified_frames", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.rectified_frames_output_options,
////            _this->_output_options.save_rectified_frames);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Stereo Rectified Frames", "");
////        PIPELINE_CTL(ctrls, _output_options.save_stereo_rectified_frames, "save_stereo_rectified_frames", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.stereo_rectified_output_options,
////            _this->_output_options.save_stereo_rectified_frames);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Quad Rectified Frames", "");
////        PIPELINE_CTL(ctrls, _output_options.save_quad_rectified_frames, "save_quad_rectified_frames", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.quad_rectified_output_options,
////            _this->_output_options.save_quad_rectified_frames);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////      PIPELINE_CTL_GROUP(ctrls, "Save Progress Video", "");
////        PIPELINE_CTL(ctrls, _output_options.save_progress_video, "save_progress_video", "");
////        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.progress_video_output_options,
////            _this->_output_options.save_progress_video);
////      PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////  }
//
//  return ctrls;
//}


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
  p->_chessboard_detection_options = this->_chessboard_detection_options;
  p->_calibration_options = this->_calibration_options;
  p->_output_options = this->_output_options;

  return true;

}

bool c_stereo_calibration_pipeline::read_stereo_frame()
{
  lock_guard lock(mutex());

  bool fOK =
      ::read_stereo_source(_input,
          _input_options.input_source.layout_type,
          _input_options.input_source.swap_cameras,
          _input_options.enable_color_maxtrix,
          _current_frames,
          _current_masks);

  if( !fOK ) {
    CF_ERROR("read_stereo_source() fails");
    return false;
  }

  for( int i = 0; i < 2; ++i ) {

    c_camera_intrinsics &intrinsics =
        _current_intrinsics.camera[i];

    if( intrinsics.image_size.empty() ) {
      intrinsics.image_size =
          _current_frames[0].size();
    }
    else if( intrinsics.image_size != _current_frames[0].size() ) {
      CF_ERROR("INPUT ERROR: Frame size change (%dx%d) -> (%dx%d) not supported\n",
          intrinsics.image_size.width, intrinsics.image_size.height,
          _current_frames[0].cols, _current_frames[0].rows);
      return false;
    }
  }

  if( _input_options.input_image_processor ) {
    for( int i = 0; i < 2; ++i ) {
      cv::Mat &image = _current_frames[i];
      cv::Mat &mask = _current_masks[i];

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

  _writing_output_videos = false;

  _output_path =
      create_output_path(_output_options.output_directory);

  _output_intrinsics_filename =
      generate_output_filename(_output_options.output_intrinsics_filename,
          "stereo_intrinsics",
          ".yml");

  _output_extrinsics_filename =
      generate_output_filename(_output_options.output_extrinsics_filename,
          "stereo_extrinsics",
          ".yml");

  /////////////////////////////////////////////////////////////////////////////

  _best_calibration_flags = _current_calibration_flags = _calibration_options.calibration_flags;
  _best_subset_quality = DBL_MAX;
  _stereo_intrinsics_initialized = false;


  if ( _chessboard_detection_options.chessboard_size.width < 2 || _chessboard_detection_options.chessboard_size.height < 2 ) {
    CF_ERROR("Invalid chessboard_detection_options_.chessboard_size: %dx%d", _chessboard_detection_options.chessboard_size.width,
        _chessboard_detection_options.chessboard_size.height);
    return false;
  }

  if ( !(_chessboard_detection_options.chessboard_cell_size.width > 0) || !(_chessboard_detection_options.chessboard_cell_size.height > 0)  ) {
    CF_ERROR("Invalid chessboard_cell_size_: %gx%g", _chessboard_detection_options.chessboard_cell_size.width,
        _chessboard_detection_options.chessboard_cell_size.height);
    return false;
  }

  _current_object_points.clear();
  _current_object_points.reserve(_chessboard_detection_options.chessboard_size.area());

  for( int i = 0; i < _chessboard_detection_options.chessboard_size.height; ++i ) {
    for( int j = 0; j < _chessboard_detection_options.chessboard_size.width; ++j ) {

      _current_object_points.emplace_back(
          j * _chessboard_detection_options.chessboard_cell_size.width,
          i * _chessboard_detection_options.chessboard_cell_size.height,
          _chessboard_detection_options.chessboard_distance);
    }
  }

  _current_extrinsics.R = cv::Matx33d::eye();
  _current_extrinsics.T = cv::Vec3d::all(0);
  _best_extrinsics.R = cv::Matx33d::eye();
  _best_extrinsics.T = cv::Vec3d::all(0);
  _new_extrinsics.R = cv::Matx33d::eye();
  _new_extrinsics.T = cv::Vec3d::all(0);

  for ( int i = 0; i < 2; ++i ) {
    _current_intrinsics.camera[i].image_size = cv::Size(0, 0);
    _current_intrinsics.camera[i].camera_matrix = cv::Matx33d::eye();
    _current_intrinsics.camera[i].dist_coeffs.clear();

    _best_intrinsics.camera[i].image_size = cv::Size(0, 0);
    _best_intrinsics.camera[i].camera_matrix = cv::Matx33d::eye();
    _best_intrinsics.camera[i].dist_coeffs.clear();

    _new_intrinsics.camera[i].image_size = cv::Size(0, 0);
    _new_intrinsics.camera[i].camera_matrix = cv::Matx33d::eye();
    _new_intrinsics.camera[i].dist_coeffs.clear();
  }

  /////////////////////////////////////////////////////////////////////////////

  const bool is_live_input =
      _input_sequence->is_live();

  if( !is_live_input ) {

    if( _input_options.input_source.left_stereo_source.empty() ) {
      CF_ERROR("ERROR: No left stereo source specified");
      return false;
    }

    if( _input_options.input_source.layout_type == stereo_frame_layout_separate_sources ) {
      if( _input_options.input_source.right_stereo_source.empty() ) {
        CF_ERROR("ERROR: No right stereo source specified");
        return false;
      }
    }

    _input.inputs[0] = _input_sequence->source(_input_options.input_source.left_stereo_source);
    if( !_input.inputs[0] ) {
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
  else if( _input_sequence->sources().empty() ) {
    CF_ERROR("ERROR: No stereo source specified");
    return false;
  }
  else {

    if( !(_input.inputs[0] = _input_sequence->source(0)) ) {
      CF_ERROR("ERROR: stereo source is null in input sequence");
      return false;
    }

    if( _input_options.input_source.layout_type == stereo_frame_layout_separate_sources ) {

      if( _input_sequence->sources().size() < 2 ) {
        CF_ERROR("ERROR: No second stereo source specified");
        return false;
      }

      if( !(_input.inputs[1] = _input_sequence->source(1)) ) {
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

  if( _chessboard_video_writer.is_open() ) {
    CF_DEBUG("closing '%s'", _chessboard_video_writer.filename().c_str());
    _chessboard_video_writer.close();
  }

  _object_points.clear();
  _current_object_points.clear();
  //  rvecs_.clear();
  //  tvecs_.clear();
  _perViewErrors.release();

  for ( int i = 0; i < 2; ++i ) {

    _image_points[i].clear();
    _current_image_points[i].clear();
    _current_frames[i].release();
    _current_masks[i].release();
    _rmaps[i].release();
  }

  base::cleanup_pipeline();
}


bool c_stereo_calibration_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if ( _current_frames[0].empty() || _current_frames[0].empty() ) {
    return false;
  }

  const cv::Size sizes[2] = {
      _current_frames[0].size(),
      _current_frames[1].size(),
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
      _current_frames[0].type());

  cv::Mat & display_frame_ =
      display_frame.getMatRef();

  _current_frames[0].copyTo(display_frame_(roi[0]));
  _current_frames[1].copyTo(display_frame_(roi[1]));

  if( _rmaps[0].empty() ) {
    _current_frames[0].copyTo(display_frame_(roi[2]));
    _current_frames[1].copyTo(display_frame_(roi[3]));
  }

  else {

    cv::remap(_current_frames[0], display_frame_(roi[2]),
        _rmaps[0], cv::noArray(),
        cv::INTER_LINEAR);

    cv::remap(_current_frames[1], display_frame_(roi[3]),
        _rmaps[1], cv::noArray(),
        cv::INTER_LINEAR);
  }

  if ( !_writing_output_videos )  {

    if( !_current_image_points[0].empty() ) {

      if ( display_frame_.channels() == 1 ) {
        cv::cvtColor(display_frame_, display_frame_,
            cv::COLOR_GRAY2BGR);
      }

      cv::drawChessboardCorners(display_frame_(roi[0]),
          _chessboard_detection_options.chessboard_size,
          _current_image_points[0],
          true);
    }

    if( !_current_image_points[1].empty() ) {

      if ( display_frame_.channels() == 1 ) {
        cv::cvtColor(display_frame_, display_frame_,
            cv::COLOR_GRAY2BGR);
      }

      cv::drawChessboardCorners(display_frame_(roi[1]),
          _chessboard_detection_options.chessboard_size,
          _current_image_points[1],
          true);
    }

    if( _rmaps[0].empty() ) {

      // draw coverage

      for ( int i = 0; i < 2; ++i ) {

        cv::Mat3b display =
            display_frame_(roi[i + 2]);

        for( const std::vector<cv::Point2f> &corners : _image_points[i] ) {
          for( const cv::Point2f &corner : corners ) {
            cv::rectangle(display, cv::Rect(corner.x - 1, corner.y - 1, 3, 3), CV_RGB(0, 255, 0), -1, cv::LINE_8);
          }
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
  ::close_stereo_source(_input);
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

  return ::open_stereo_source(_input, _input_options.input_source.layout_type);
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

  return ::seek_stereo_source(_input, pos);
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
            _input.inputs[0]->size() :
            std::min(_input.inputs[0]->size(),
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
      _calibration_options.enable_calibration &&
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
        _object_points.size();

    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  close_input_source();

  if ( canceled()  ) {
    return false;
  }

  if ( _calibration_options.enable_calibration && !_input_sequence->is_live()  ) {

    if ( !update_calibration() ) {
      CF_ERROR("update_stereo_calibration() fails");
      return false;
    }

    if ( canceled()  ) {
      return false;
    }

    _writing_output_videos = true;
    if ( !write_output_videos() ) {
      return false;
    }
  }

  return true;
}

bool c_stereo_calibration_pipeline::update_calibration()
{
  if( !_stereo_intrinsics_initialized && _calibration_options.init_camera_matrix_2d ) {

    const cv::Size image_size =
        _current_frames[0].size();

      _stereo_intrinsics_initialized =
          init_camera_intrinsics(_current_intrinsics,
              _object_points,
              _image_points[0],
              _image_points[1],
              image_size,
              1);

    if( !_stereo_intrinsics_initialized ) {
      CF_ERROR("init_camera_intrinsics() fails");
      return false; // wait for next frame
    }

    const cv::Matx33d &M0 =
        _current_intrinsics.camera[0].camera_matrix;

    const cv::Matx33d &M1 =
        _current_intrinsics.camera[1].camera_matrix;

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

  if( _best_subset_quality < DBL_MAX ) {
    _current_intrinsics = _best_intrinsics;
    _current_extrinsics = _best_extrinsics;
    _current_calibration_flags = _best_calibration_flags;
  }

  CF_DEBUG("Run stereo_calibrate() ...");

  _rmse =
      stereo_calibrate(_object_points,
          _image_points[0], _image_points[1],
          _current_intrinsics,
          _current_extrinsics,
          _current_calibration_flags,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
              _calibration_options.max_iterations,
              _calibration_options.solver_eps),
          &_E,
          &_F,
          // &rvecs_,
          // &tvecs_,
          &_perViewErrors);

  const cv::Matx33d &M0 =
      _current_intrinsics.camera[0].camera_matrix;

  const cv::Matx33d &M1 =
      _current_intrinsics.camera[1].camera_matrix;

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
      _rmse);

  if( _rmse >= 0 ) {

    const double subset_quality =
        estimate_subset_quality();

    _stereo_intrinsics_initialized = true;

    CF_DEBUG("subset_quality=%g best_subset_quality_=%g",
        subset_quality, _best_subset_quality);

    if( subset_quality < _best_subset_quality ) {
      _best_intrinsics = _current_intrinsics;
      _best_extrinsics = _current_extrinsics;
      _best_calibration_flags = _current_calibration_flags;
      _best_subset_quality = subset_quality;

      update_state();

      if( !save_current_camera_parameters() ) {
        CF_ERROR("save_current_camera_parameters() fails");
        return false;
      }

      update_undistortion_remap();

      return true;
    }
    else {
      CF_DEBUG("NOT ASSIGN _best_subset_quality=%g subset_quality=%g", _best_subset_quality, subset_quality);
    }
  }


  return false;
}


double c_stereo_calibration_pipeline::estimate_subset_quality() const
{
  const int nbframes =
      _object_points.size();

  if( nbframes > 0 ) {

    double grid_mean, grid_stdev;

    estimate_grid_meanstdev(&grid_mean, &grid_stdev, nbframes);

    const double grid_quality =
        grid_stdev / grid_mean;


    double rmse_quality = 0;
    for( size_t i = 0; i < nbframes; i++ ) {
      rmse_quality += (_perViewErrors[i][0] + _perViewErrors[i][1]);
    }
    rmse_quality /= nbframes;


    const double alpha =
        _calibration_options.filter_alpha;

    CF_DEBUG("grid: mean=%g stdev=%g quality=%g ; rmse: %g",
        grid_mean, grid_stdev, grid_quality, rmse_quality);

    return 0.5 * rmse_quality * alpha + grid_quality * (1 - alpha);
  }

  return 0;
}

void c_stereo_calibration_pipeline::estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const
{
  const cv::Size image_size = _current_frames[0].size();

  const int gridSize = 10;
  const int xGridStep = image_size.width / gridSize;
  const int yGridStep = image_size.height / gridSize;
  const int stride = gridSize * gridSize;

  std::vector<int> pointsInCell(2 * stride);

  std::fill(pointsInCell.begin(), pointsInCell.end(), 0);

  for( int k = 0; k < _object_points.size(); k++ )
    if( k != excludedIndex ) {

      for( int i = 0; i < 2; ++i ) {

        for( const auto &p : _image_points[i][k] ) {

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
      _object_points.size();

  const int nbframesmax =
      std::max(1, std::max(_calibration_options.min_frames,
          _calibration_options.max_frames));

  //CF_DEBUG("nbframes = %d / %d", nbframes, nbframesmax);

  if( !only_landmarks && nbframes != _perViewErrors.rows ) {

    if( nbframes > 1 ) {
      CF_ERROR("APP BUG: object_points_.size()=%zu != current_per_view_errors_.total()=%d",
          _object_points.size(),
          _perViewErrors.rows);
    }

    if( nbframes > nbframesmax ) {
      _image_points[0].erase(_image_points[0].begin());
      _image_points[1].erase(_image_points[1].begin());
      _object_points.erase(_object_points.begin());
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
        _calibration_options.filter_alpha;

    const double totalRmeQuality =
        only_landmarks ? 0 :
            estimateRmeQuality(_perViewErrors);

    const double totalCoverageQuality =
        estimate_coverage_quality(nbframes);

    double bestQuality = DBL_MAX;
    int worstElemIndex = 0;

    for( int i = 0; i < nbframes; ++i ) {

      const double currentCoverageQuality =
          estimate_coverage_quality(i);

      double currentQuality;

      if ( only_landmarks ) {
        currentQuality =
            currentCoverageQuality;
      }
      else {

        const double currentRmseQuality =
            (totalRmeQuality - 0.5 * (_perViewErrors[i][0] + _perViewErrors[i][1])) / (_perViewErrors.rows - 1);

        currentQuality =
            alpha * currentRmseQuality + (1. - alpha) * currentCoverageQuality;
      }

      if( currentQuality < bestQuality ) {
        bestQuality = currentQuality;
        worstElemIndex = i;
      }
    }

    // CF_DEBUG("worstElemIndex=%d bestSubsetQuality=%g", worstElemIndex, bestSubsetQuality);

    _image_points[0].erase(_image_points[0].begin() + worstElemIndex);
    _image_points[1].erase(_image_points[1].begin() + worstElemIndex);
    _object_points.erase(_object_points.begin() + worstElemIndex);

    if ( nbframes == _perViewErrors.rows ) {

      cv::Mat1f newErrorsVec(nbframes - 1,
          _perViewErrors.cols);

      std::copy(_perViewErrors[0],
          _perViewErrors[worstElemIndex],
          newErrorsVec[0]);

      if( worstElemIndex < nbframes - 1 ) {
        std::copy(_perViewErrors[worstElemIndex + 1],
            _perViewErrors[nbframes],
            newErrorsVec[worstElemIndex]);
      }

      _perViewErrors = newErrorsVec;
    }
  }
}

void c_stereo_calibration_pipeline::update_state()
{
  if( _calibration_options.auto_tune_calibration_flags && _object_points.size() > _calibration_options.min_frames ) {

    if( !(_current_calibration_flags & cv::CALIB_ZERO_TANGENT_DIST) ) {

      const double eps = 0.005;

      bool fix_zero_tangent_dist = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            _current_intrinsics.camera[i].dist_coeffs;

        if( (D.size() > 3) && (fabs(D[2]) > eps || fabs(D[3]) > eps) ) {
          fix_zero_tangent_dist = false;
          break;
        }
      }

      if( fix_zero_tangent_dist ) {
        _current_calibration_flags |= cv::CALIB_ZERO_TANGENT_DIST;
      }
    }

    if( !(_current_calibration_flags & cv::CALIB_FIX_K1) ) {

      const double eps = 0.005;

      bool fix_k1 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            _current_intrinsics.camera[i].dist_coeffs;

        if( (D.size() > 0) && (fabs(D[0]) > eps) ) {
          fix_k1 = false;
          break;
        }
      }

      if( fix_k1 ) {
        _current_calibration_flags |= cv::CALIB_FIX_K1;
      }
    }

    if( !(_current_calibration_flags & cv::CALIB_FIX_K2) ) {

      const double eps = 0.005;

      bool fix_k2 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            _current_intrinsics.camera[i].dist_coeffs;

        if( (D.size() > 1) && (fabs(D[1]) > eps) ) {
          fix_k2 = false;
          break;
        }
      }

      if( fix_k2 ) {
        _current_calibration_flags |= cv::CALIB_FIX_K2;
      }
    }

    if( !(_current_calibration_flags & cv::CALIB_FIX_K3) ) {

      const double eps = 0.005;

      bool fix_k3 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            _current_intrinsics.camera[i].dist_coeffs;

        if( (D.size() > 4) && (fabs(D[4]) > eps) ) {
          fix_k3 = false;
          break;
        }
      }

      if( fix_k3 ) {
        _current_calibration_flags |= cv::CALIB_FIX_K3;
      }
    }
  }
}


void c_stereo_calibration_pipeline::update_undistortion_remap()
{
  const cv::Size & image_size =
      _best_intrinsics.camera[0].image_size;

  create_stereo_rectification(image_size,
      _best_intrinsics,
      _best_extrinsics,
      -1,
      _rmaps,
      &_new_intrinsics,
      &_new_extrinsics,
      nullptr,
      nullptr,
      nullptr,
      nullptr);
}

bool c_stereo_calibration_pipeline::save_current_camera_parameters() const
{
  if( !_output_intrinsics_filename.empty() ) {

    if( !create_path(get_parent_directory(_output_intrinsics_filename)) ) {
      CF_ERROR("create_path('%s') fails: %s", _output_intrinsics_filename.c_str(), strerror(errno));
      return false;
    }

    CF_DEBUG("saving output_intrinsics_filename_: %s", _output_intrinsics_filename.c_str());

    if( !write_stereo_camera_intrinsics_yml(_best_intrinsics, _output_intrinsics_filename) ) {
      CF_ERROR("ERROR: save_stereo_camera_intrinsics_yml('%s') fails",
          _output_intrinsics_filename.c_str());
      return false;
    }
  }

  if( !_output_extrinsics_filename.empty() ) {

    if( !create_path(get_parent_directory(_output_extrinsics_filename)) ) {
      CF_ERROR("create_path('%s') fails: %s", _output_extrinsics_filename.c_str(), strerror(errno));
      return false;
    }

    CF_DEBUG("saving output_extrinsics_filename_: %s", _output_extrinsics_filename.c_str());

    if( !write_stereo_camera_extrinsics_yml(_best_extrinsics, _output_extrinsics_filename) ) {
      CF_ERROR("ERROR: save_stereo_camera_extrinsics_yml('%s') fails",
          _output_extrinsics_filename.c_str());
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
      _chessboard_detection_options.chessboard_size,
      corners_,
      _chessboard_detection_options);
}

bool c_stereo_calibration_pipeline::process_current_stereo_frame(bool enable_calibration)
{

  if ( true ) {

    lock_guard lock(mutex());

    for( int i = 0; i < 2; ++i ) {
      _current_image_points[i].clear();
    }

    for( int i = 0; i < 2; ++i ) {
      if( !detect_chessboard(_current_frames[i], _current_image_points[i]) ) {
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

    _image_points[0].emplace_back(_current_image_points[0]);
    _image_points[1].emplace_back(_current_image_points[1]);
    _object_points.emplace_back(_current_object_points);

    if( _object_points.size() >= std::max(1, _calibration_options.min_frames) ) {

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
  if( !_output_options.save_chessboard_frames ) {
    return true;
  }

  const cv::Size sizes[2] = {
      _current_frames[0].size(),
      _current_frames[1].size(),
  };

  const cv::Size totalsize(sizes[0].width + sizes[1].width,
      std::max(sizes[0].height, sizes[1].height));

  const cv::Rect roi[2] = {
      cv::Rect(0, 0, sizes[0].width, sizes[0].height),
      cv::Rect(sizes[0].width, 0, sizes[1].width, sizes[1].height),
  };

  cv::Mat frame(totalsize, _current_frames[0].type());
  _current_frames[0].copyTo(frame(roi[0]));
  _current_frames[1].copyTo(frame(roi[1]));

  if( !_chessboard_video_writer.is_open() ) {

    bool fOK =
        open_output_writer(_chessboard_video_writer,
            _output_options.chessboard_frames_output_options,
            "chessboard",
            ".avi");

    if( !fOK ) {
      CF_ERROR("chessboard_video_writer_.open('%s') fails",
          _chessboard_video_writer.filename().c_str());
      return false;
    }

    CF_DEBUG("Created '%s'", _chessboard_video_writer.filename().c_str());
  }

  if( !_chessboard_video_writer.write(frame, cv::noArray(), false, _input_sequence->current_pos() - 1) ) {
    CF_ERROR("chessboard_video_writer_.write() fails");
    return false;
  }

  return true;
}

bool c_stereo_calibration_pipeline::write_output_videos()
{
  if( !_output_options.save_rectified_frames &&
      !_output_options.save_stereo_rectified_frames &&
      !_output_options.save_quad_rectified_frames &&
      !_output_options.save_quad_output_frames ) {
    return true;
  }


  CF_DEBUG("update_undistortion_remap()...");
  update_undistortion_remap();

  CF_DEBUG("Save rectified videos...");

  c_output_frame_writer video_writer[2];
  c_output_frame_writer stereo_writer;
  c_output_frame_writer quad_writer;
  c_output_frame_writer quad_rectified_writer;

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

  _total_frames = _input.inputs[0]->size();
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
      cv::remap(_current_frames[i], remapped_frames[i],
          _rmaps[i], cv::noArray(),
          cv::INTER_LINEAR);
    }

    sizes[0] = remapped_frames[0].size();
    sizes[1] = remapped_frames[1].size();

    ///////////////////////////////////////////////////////////////
    if( _output_options.save_rectified_frames ) {

      for( int i = 0; i < 2; ++i ) {

        if( !video_writer[i].is_open() ) {

          fOK =
              open_output_writer(video_writer[i],
                  _output_options.rectified_frames_output_options,
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

    ///////////////////////////////////////////////////////////////
    if( _output_options.save_stereo_rectified_frames ) {

      if( !stereo_writer.is_open() ) {

        stereo_size.width = sizes[0].width + sizes[1].width;
        stereo_size.height = std::max(sizes[0].height, sizes[1].height);

        fOK =
            open_output_writer(stereo_writer,
                _output_options.stereo_rectified_output_options,
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

    ///////////////////////////////////////////////////////////////
    if ( _output_options.save_quad_output_frames ) {

      if( !quad_writer.is_open() ) {

        quad_size.width = sizes[0].width + sizes[1].width;
        quad_size.height = sizes[0].height + sizes[1].height;

        fOK =
            open_output_writer(quad_writer,
                _output_options.quad_output_options,
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
      _current_frames[0].copyTo(display_frame(rc0));
      _current_frames[1].copyTo(display_frame(rc1));
      remapped_frames[0].copyTo(display_frame(rc2));
      remapped_frames[1].copyTo(display_frame(rc3));

      if ( !quad_writer.write(display_frame, cv::noArray(), false, _processed_frames ) ) {
        CF_ERROR("quad_writer.write() fails");
        return false;
      }
    }

    ///////////////////////////////////////////////////////////////
    if ( _output_options.save_quad_rectified_frames ) {

      if( !quad_rectified_writer.is_open() ) {

        quad_size.width = sizes[0].width + sizes[1].width;
        quad_size.height = sizes[0].height + sizes[1].height;

        fOK =
            open_output_writer(quad_rectified_writer,
                _output_options.quad_rectified_output_options,
                "quad_rect",
                ".avi");

        if( !fOK ) {
          CF_ERROR("quad_rectified_writer.open() fails");
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

      if ( !quad_rectified_writer.write(display_frame, cv::noArray(), false, _processed_frames ) ) {
        CF_ERROR("quad_rectified_writer.write() fails");
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


