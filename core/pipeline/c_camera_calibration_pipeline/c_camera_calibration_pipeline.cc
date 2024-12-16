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
  return _input_options;
}

const c_camera_calibration_input_options & c_camera_calibration_pipeline::input_options() const
{
  return _input_options;
}

c_chessboard_corners_detection_options & c_camera_calibration_pipeline::chessboard_corners_detection_options()
{
  return _chessboard_detection_options;
}

const c_chessboard_corners_detection_options & c_camera_calibration_pipeline::chessboard_corners_detection_options() const
{
  return _chessboard_detection_options;
}

c_calibrate_camera_options & c_camera_calibration_pipeline::calibrate_camera_options()
{
  return _calibration_options;
}

const c_calibrate_camera_options & c_camera_calibration_pipeline::calibrate_camera_options() const
{
  return _calibration_options;
}

c_camera_calibration_output_options& c_camera_calibration_pipeline::output_options()
{
  return _output_options;
}

const c_camera_calibration_output_options& c_camera_calibration_pipeline::output_options() const
{
  return _output_options;
}

bool c_camera_calibration_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if ( _current_frame.empty() ) {
    return false;
  }

  const cv::Size totalSize(_current_frame.cols * 2,
      _current_frame.rows);

  const cv::Rect roi[2] = {
      cv::Rect(0, 0, _current_frame.cols, _current_frame.rows),
      cv::Rect(_current_frame.cols, 0, _current_frame.cols, _current_frame.rows),
  };

  display_frame.create(totalSize,
      CV_MAKETYPE(_current_frame.depth(), 3));

  cv::Mat & display_frame_ =
      display_frame.getMatRef();

  if( _current_frame.channels() == 3 ) {
    _current_frame.copyTo(display_frame_(roi[0]));
  }
  else {
    cv::cvtColor(_current_frame, display_frame_(roi[0]),
        cv::COLOR_GRAY2BGR);
  }

  if ( _is_chessboard_found ) {

    cv::drawChessboardCorners(display_frame_(roi[0]),
        _chessboard_detection_options.chessboard_size,
        _current_image_points,
        _is_chessboard_found);
  }

  if( !_current_undistortion_remap.empty() ) {

    if( display_frame_.channels() == _current_frame.channels() ) {

        cv::remap(_current_frame, display_frame_(roi[1]),
            _current_undistortion_remap, cv::noArray(),
            cv::INTER_LINEAR);
    }
    else {
      cv::Mat tmp;

      cv::remap(_current_frame, tmp,
          _current_undistortion_remap, cv::noArray(),
          cv::INTER_LINEAR);

      cv::cvtColor(tmp, display_frame_(roi[1]),
          cv::COLOR_GRAY2BGR);
    }

  }
  else {

    // draw coverage

    if( _current_frame.channels() == 3 ) {
      _current_frame.copyTo(display_frame_(roi[1]));
    }
    else {
      cv::cvtColor(_current_frame, display_frame_(roi[1]),
          cv::COLOR_GRAY2BGR);
    }

    cv::Mat3b display =
        display_frame_(roi[1]);

    for( const std::vector<cv::Point2f> &corners : _image_points ) {
      for( const cv::Point2f &corner : corners ) {

        cv::rectangle(display, cv::Rect(corner.x - 1, corner.y - 1, 3, 3),
            CV_RGB(0, 255, 0), -1,
            cv::LINE_8);
      }
    }
  }

  if ( display_frame.needed() ) {
    display_frame_.copyTo(display_frame);
  }
  if ( display_mask.needed() ) {
    display_mask.release();
    // display_mask_.copyTo(display_mask);
  }
  return true;
}

bool c_camera_calibration_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
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
    SERIALIZE_OPTION(section, save, _calibration_options, max_iterations);
    SERIALIZE_OPTION(section, save, _calibration_options, solver_eps);
    SERIALIZE_OPTION(section, save, _calibration_options, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);

    SERIALIZE_OPTION(section, save, _output_options, save_chessboard_frames);
    if ( (subsection = SERIALIZE_GROUP(section, save, "output_chessboard_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_chessboard_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_rectified_frames);
    if ( (subsection = SERIALIZE_GROUP(section, save, "output_rectified_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_rectified_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_debug_video);
    if ( (subsection = SERIALIZE_GROUP(section, save, "output_debug_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_debug_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_coverage_frame);
    SERIALIZE_OPTION(section, save, _output_options, output_coverage_frame_filename);

  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl> & c_camera_calibration_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls); // , _this->_input_options

    PIPELINE_CTL_GROUP(ctrls, "Chessboard corners detection", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.method, "Method", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.chessboard_size, "chessboard_size", "Size of chessboard - number of internal chessboard corners");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.chessboard_cell_size, "chessboard_cell_size", "cell size in [m]");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.chessboard_distance, "chessboard_distance", "distance to chessboard in [m]");


      PIPELINE_CTL_GROUP(ctrls, "findChessboardCorners", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options. findChessboardCorners.max_scales, "max_scales", "See documentation to cv::findChessboardCorners()");
      PIPELINE_CTL_BITFLAGS(ctrls, _chessboard_detection_options.findChessboardCorners.flags, FindChessboardCornersFlags,"flags", "See documentation to cv::findChessboardCorners()");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "findChessboardCornersSB", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.findChessboardCornersSB.max_scales, "max_scales", "See documentation to cv::findChessboardCornersSB()");
      PIPELINE_CTL_BITFLAGS(ctrls, _chessboard_detection_options.findChessboardCornersSB.flags,FindChessboardCornersSBFlags, "flags", "See documentation to cv::findChessboardCornersSB()");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "cornerSubPix options", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.winSize, "winSize", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.zeroZone, "zeroZone", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.max_solver_iterations, "max_solver_iterations", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.cornerSubPix.solver_eps, "solver_eps", "");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "BilateralFilter options", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.bilateralFilter.d, "d", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.bilateralFilter.sigmaColor, "sigmaColor", "");
      PIPELINE_CTL(ctrls, _chessboard_detection_options.bilateralFilter.sigmaSpace, "sigmaSpace", "");
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Calibration options", "");
    PIPELINE_CTL(ctrls,  _calibration_options.enable_calibration, "enable_calibration", "");
    PIPELINE_CTL(ctrls,  _calibration_options.min_frames, "min_frames", "");
    PIPELINE_CTL(ctrls,  _calibration_options.max_frames, "max_frames", "");
    PIPELINE_CTL_BITFLAGS(ctrls, _calibration_options.calibration_flags, CAMERA_CALIBRATION_FLAGS,  "calibration flags", "" );
    PIPELINE_CTL(ctrls,  _calibration_options.auto_tune_calibration_flags, "auto_tune_calibration_flags", "");
    PIPELINE_CTL(ctrls,  _calibration_options.init_camera_matrix_2d, "init_camera_matrix_2d", "");
    PIPELINE_CTL(ctrls,  _calibration_options.max_iterations, "max_iterations", "");
    PIPELINE_CTL(ctrls,  _calibration_options.solver_eps, "solver_eps", "");
    PIPELINE_CTL(ctrls,  _calibration_options.filter_alpha, "filter_alpha", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
      PIPELINE_CTL(ctrls, _output_options.default_display_type, "display_type", "");
      PIPELINE_CTL(ctrls, _output_options.output_intrinsics_filename, "intrinsics_filename", "");

      PIPELINE_CTL(ctrls, _output_options.save_coverage_frame, "save_coverage_frame", "");
      PIPELINE_CTLC(ctrls, _output_options.output_coverage_frame_filename, "coverage_frame_filename", "", _this->_output_options.save_coverage_frame);

      PIPELINE_CTL_GROUP(ctrls, "Save Chessboard Frames", "");
        PIPELINE_CTL(ctrls, _output_options.save_chessboard_frames, "save_chessboard_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_chessboard_video_options,
            _this->_output_options.save_chessboard_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "Save Rectified Frames", "");
        PIPELINE_CTL(ctrls, _output_options.save_rectified_frames, "save_rectified_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_rectified_video_options,
            _this->_output_options.save_rectified_frames);
      PIPELINE_CTL_END_GROUP(ctrls);


      PIPELINE_CTL_GROUP(ctrls, "Save Debug video", "");
        PIPELINE_CTL(ctrls, _output_options.save_debug_video, "save_debug_video", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_debug_video_options, _this->_output_options.save_debug_video);
      PIPELINE_CTL_END_GROUP(ctrls);



    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

bool c_camera_calibration_pipeline::copyParameters(const base::sptr & dst) const
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


bool c_camera_calibration_pipeline::read_input_frame(const c_input_sequence::sptr & input_sequence,
    cv::Mat & output_image, cv::Mat & output_mask)
{
  lock_guard lock(mutex());

  // input_sequence->set_auto_debayer(DEBAYER_DISABLE);
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
            algo, toCString(algo));
        return false;
    }
  }

  if( _input_options.enable_color_maxtrix && input_sequence->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        input_sequence->color_matrix());
  }

//  if ( anscombe_.method() != anscombe_none ) {
//    anscombe_.apply(output_image, output_image);
//  }

  if ( !_missing_pixel_mask.empty() ) {

    if ( output_image.size() != _missing_pixel_mask.size() ) {

      CF_ERROR("Invalid input: "
          "frame and bad pixel mask sizes not match:\n"
          "frame size: %dx%d\n"
          "mask size : %dx%d",
          output_image.cols, output_image.rows,
          _missing_pixel_mask.cols, _missing_pixel_mask.rows);

      return false;
    }

    if ( output_mask.empty() ) {
      _missing_pixel_mask.copyTo(output_mask);
    }
    else {
      cv::bitwise_and(output_mask, _missing_pixel_mask,
          output_mask);
    }
  }

  if ( !output_mask.empty() && _input_options.inpaint_missing_pixels ) {
    linear_interpolation_inpaint(output_image, output_mask, output_image);
  }

  return true;
}

bool c_camera_calibration_pipeline::initialize_pipeline()
{
   if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
    return false;
  }

  _output_path =
      create_output_path(_output_options.output_directory);

  _output_intrinsics_filename =
      generate_output_filename(_output_options.output_intrinsics_filename,
          "camera_intrinsics",
          ".yml");

  _chessboard_video_writer.close();
  _current_image_points.clear();
  _current_object_points.clear();
  _stdDeviations.release();
  _perViewErrors.release();
  _current_undistortion_remap.release();
  _image_points.clear();
  _object_points.clear();
  _intrinsics_initialized = false;
  _is_chessboard_found = false;
  _confIntervalsState = false;

  _current_calibration_flags = _calibration_options.calibration_flags;
  _best_subset_quality = HUGE_VAL;

  _current_intrinsics.camera_matrix = cv::Matx33d::zeros();
  _current_intrinsics.dist_coeffs.clear();

  if ( _chessboard_detection_options.chessboard_size.width < 2 || _chessboard_detection_options.chessboard_size.height < 2 ) {
    CF_ERROR("Invalid chessboard_size_: %dx%d", _chessboard_detection_options.chessboard_size.width,
        _chessboard_detection_options.chessboard_size.height);
    return false;
  }

  if ( !(_chessboard_detection_options.chessboard_cell_size.width > 0) || !(_chessboard_detection_options.chessboard_cell_size.height > 0)  ) {
    CF_ERROR("Invalid chessboard_cell_size_: %gx%g", _chessboard_detection_options.chessboard_cell_size.width,
        _chessboard_detection_options.chessboard_cell_size.height);
    return false;
  }

  _current_object_points.reserve(_chessboard_detection_options.chessboard_size.area());

  for( int i = 0; i < _chessboard_detection_options.chessboard_size.height; ++i ) {
    for( int j = 0; j < _chessboard_detection_options.chessboard_size.width; ++j ) {

      _current_object_points.emplace_back(
          j * _chessboard_detection_options.chessboard_cell_size.width,
          i * _chessboard_detection_options.chessboard_cell_size.height,
          _chessboard_detection_options.chessboard_distance);
    }
  }

  return true;
}

void c_camera_calibration_pipeline::cleanup_pipeline()
{
  close_input_sequence();

  if( _chessboard_video_writer.is_open() ) {
    CF_DEBUG("Closing '%s'", _chessboard_video_writer.filename().c_str());
    _chessboard_video_writer.close();
  }

  _current_image_points.clear();
  _current_object_points.clear();
  _stdDeviations.release();
  _perViewErrors.release();
  _current_undistortion_remap.release();
  _image_points.clear();
  _object_points.clear();
  _intrinsics_initialized = false;
  _is_chessboard_found = false;
  _confIntervalsState = false;

  base::cleanup_pipeline();
}

bool c_camera_calibration_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());


  ///////////////////////////////////

  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }

  if ( _input_sequence->is_live() ) {

    _total_frames = INT_MAX;
    _processed_frames = 0;
    _accumulated_frames = 0;

  }
  else {

    const int start_pos =
        std::max(_input_options.start_frame_index, 0);

    const int end_pos =
        _input_options.max_input_frames < 1 ?
            _input_sequence->size() :
            std::min(_input_sequence->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);


    _total_frames = end_pos - start_pos;
    _processed_frames = 0;
    _accumulated_frames = 0;

    if( _total_frames < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
          _total_frames);
      return false;
    }

    if( !seek_input_sequence(start_pos) ) {
      CF_ERROR("ERROR: seek_input_source(start_pos=%d) fails", start_pos);
      return false;
    }
  }


  set_status_msg("RUNNING ...");

  const bool is_live_sequence =
      _input_sequence->is_live();

  const bool enable_live_calibration =
      is_live_sequence &&
          _calibration_options.enable_calibration;

  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    if( !is_live_sequence && is_bad_frame_index(_input_sequence->current_pos()) ) {
      CF_DEBUG("Skip frame %d as blacklisted", _input_sequence->current_pos());
      _input_sequence->seek(_input_sequence->current_pos() + 1);
      continue;
    }

    if ( canceled() ) {
      break;
    }

    if( !read_input_frame(_input_sequence, _current_frame, _current_mask) ) {
      set_status_msg("read_input_frame() fails");
      break;
    }

    if ( canceled() ) {
      break;
    }

    if( _input_options.input_image_processor ) {
      lock_guard lock(mutex());
      if( !_input_options.input_image_processor->process(_current_frame, _current_mask) ) {
        CF_ERROR("ERROR: input_image_processor->process(current_frame_) fails");
        return false;
      }
      if ( canceled() ) {
        break;
      }
    }


    if ( !process_current_frame(enable_live_calibration) ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    _accumulated_frames =
          _object_points.size();


    // give chance GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if ( canceled() ) {
      break;
    }
  }

  close_input_sequence();


  if ( !is_live_sequence && _calibration_options.enable_calibration ) {

    CF_DEBUG("update_calibration()...");

    if ( !update_calibration() ) {
      CF_ERROR("update_calibration() fails");
      return false;
    }

    if ( !write_output_videos() ) {
      return false;
    }
  }

  return  true;
}

bool c_camera_calibration_pipeline::process_current_frame(bool enable_calibration)
{
  if( _current_frame.empty() || !detect_chessboard(_current_frame) ) {
    return true; // wait for next frame
  }

  if( !write_chessboard_video() ) {
    CF_ERROR("write_chessboard_video() fails");
    return false;
  }

  lock_guard lock(mutex());

  _image_points.emplace_back(_current_image_points);
  _object_points.emplace_back(_current_object_points);

  if( _image_points.size() >= _calibration_options.min_frames ) {

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

  return true;
}

bool c_camera_calibration_pipeline::detect_chessboard(const cv::Mat & frame)
{
  lock_guard lock(mutex());
  _is_chessboard_found =
      find_chessboard_corners(frame,
          _chessboard_detection_options.chessboard_size,
          _current_image_points,
          _chessboard_detection_options);

  return _is_chessboard_found;
}

void c_camera_calibration_pipeline::filter_frames(bool only_landmarks)
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
            sum += perViewErrors[i][0];
          }
          return sum;
        };


    const double alpha =
        _calibration_options.filter_alpha;

    const double totalRmeQuality =
        only_landmarks ? 0 :
            estimateRmeQuality(_perViewErrors);

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
            (totalRmeQuality - _perViewErrors[i][0]) / (_perViewErrors.rows - 1);

        currentSubsetQuality =
            alpha * currentRmseQuality + (1. - alpha) * currentCoverageQuality;
      }

      if( currentSubsetQuality < bestSubsetQuality ) {
        bestSubsetQuality = currentSubsetQuality;
        worstElemIndex = i;
      }
    }

    // CF_DEBUG("worstElemIndex=%d bestSubsetQuality=%g", worstElemIndex, bestSubsetQuality);

    _image_points.erase(_image_points.begin() + worstElemIndex);
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

double c_camera_calibration_pipeline::estimate_subset_quality() const
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

double c_camera_calibration_pipeline::estimate_coverage_quality(int excludedIndex) const
{
  double mean, stdev;

  estimate_grid_meanstdev(&mean, &stdev,
      excludedIndex);

  return stdev / mean;
}

void c_camera_calibration_pipeline::estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const
{
  int gridSize = 10;
  int xGridStep = _current_frame.cols / gridSize;
  int yGridStep = _current_frame.rows / gridSize;

  std::vector<int> pointsInCell(gridSize * gridSize);

  std::fill(pointsInCell.begin(), pointsInCell.end(), 0);

  for( size_t k = 0; k < _image_points.size(); k++ )
    if( k != excludedIndex ) {

      for( auto pointIt = _image_points[k].begin(); pointIt != _image_points[k].end(); ++pointIt ) {

        int i = (int) ((*pointIt).x / xGridStep);
        int j = (int) ((*pointIt).y / yGridStep);
        pointsInCell[i * gridSize + j]++;
      }
    }

  cv::Scalar mean, stdev;
  cv::meanStdDev(pointsInCell, mean, stdev);

  if ( m ) {
    *m = mean[0];
  }

  if ( s ) {
    *s = stdev[0];
  }
}

bool c_camera_calibration_pipeline::update_calibration()
{
  const cv::Size image_size =
      _current_frame.size();

  if( !_intrinsics_initialized ) {

    if( !_calibration_options.init_camera_matrix_2d ) {

      _current_intrinsics.image_size = image_size;
      _current_intrinsics.camera_matrix = cv::Matx33d::zeros();
      _current_intrinsics.dist_coeffs.clear();
    }

    else {

      _intrinsics_initialized =
          init_camera_intrinsics(_current_intrinsics,
              _object_points,
              _image_points,
              image_size);

      if( !_intrinsics_initialized ) {
        CF_ERROR("init_camera_intrinsics() fails");
        // update_display_image();
        return true; // wait for next frame
      }

      if( canceled() ) {
        return false;
      }
    }

    _best_intrinsics = _current_intrinsics;
    _best_calibration_flags = _current_calibration_flags;
  }

  if ( _best_subset_quality < HUGE_VAL ) {
    _current_intrinsics = _best_intrinsics;
    _current_calibration_flags = _best_calibration_flags;
  }

  _rmse =
      calibrate_camera(_object_points,
          _image_points,
          _current_intrinsics,
          _current_calibration_flags,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
              _calibration_options.max_iterations,
              _calibration_options.solver_eps),
          nullptr,
          nullptr,
          &_stdDeviations,
          nullptr,
          &_perViewErrors);

  const cv::Matx33d & M =
      _current_intrinsics.camera_matrix;

  CF_DEBUG("\n"
      "M: {\n"
      "  %+g %+g %+g\n"
      "  %+g %+g %+g\n"
      "  %+g %+g %+g\n"
      "}\n"
      "rmse = %g\n",
      M(0, 0), M(0, 1), M(0, 2),
      M(1, 0), M(1, 1), M(1, 2),
      M(2, 0), M(2, 1), M(2, 2),
      _rmse);

  if ( canceled() ) {
    return false;
  }

  if( _rmse >= 0 ) {

    _intrinsics_initialized = true;

    const double subset_quality =
        estimate_subset_quality();

    CF_DEBUG("subset_quality=%g best_subset_quality_=%g",
        subset_quality, _best_subset_quality);

    if( subset_quality < _best_subset_quality ) {

      _best_intrinsics = _current_intrinsics;
      _best_calibration_flags = _current_calibration_flags;
      _best_subset_quality = subset_quality;

      update_state();

      if( canceled() ) {
        return false;
      }

      if( !save_current_camera_parameters() ) {
        CF_ERROR("save_current_camera_parameters() fails");
        return false;
      }

      update_undistortion_remap();

      if( canceled() ) {
        return false;
      }
    }
  }

  return true;
}


void c_camera_calibration_pipeline::update_state()
{
  cv::Matx33d &cameraMatrix =
      _current_intrinsics.camera_matrix;

  if( true) {

    const double relErrEps = 0.05;
    static const double sigmaMult = 1.96;

    bool fConfState = false;
    bool cConfState = false;
    bool dConfState = true;

    const cv::Mat &S =
        _stdDeviations;

    if( sigmaMult * S.at<double>(0) / cameraMatrix(0, 0) < relErrEps &&
        sigmaMult * S.at<double>(1) / cameraMatrix(1, 1) < relErrEps ) {
      fConfState = true;
    }

    if( sigmaMult * S.at<double>(2) / cameraMatrix(0, 2) < relErrEps &&
        sigmaMult * S.at<double>(3) / cameraMatrix(1, 2) < relErrEps ) {
      cConfState = true;
    }

    for( int i = 0; i < 5; i++ ) {
      if( S.at<double>(4 + i) / fabs(_current_intrinsics.dist_coeffs[i]) > 1 ) {
        dConfState = false;
      }
    }

    _confIntervalsState =
        fConfState && cConfState && dConfState;
  }

  if( _calibration_options.auto_tune_calibration_flags && _image_points.size() > _calibration_options.min_frames ) {

    if( !(_current_calibration_flags & cv::CALIB_FIX_ASPECT_RATIO) ) {

      const double fDiff =
          fabs(cameraMatrix(0, 0) - cameraMatrix(1, 1));

      const cv::Mat &S =
          _stdDeviations;

      if( fDiff < 3 * S.at<double>(0) && fDiff < 3 * S.at<double>(1) ) {
        _current_calibration_flags |= cv::CALIB_FIX_ASPECT_RATIO;
        cameraMatrix(0, 0) = cameraMatrix(1, 1);
      }
    }

    if( !(_current_calibration_flags & cv::CALIB_ZERO_TANGENT_DIST) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          _current_intrinsics.dist_coeffs;

      if( D.size() > 3 && fabs(D[2]) < eps && fabs(D[3]) < eps ) {
        _current_calibration_flags |= cv::CALIB_ZERO_TANGENT_DIST;
      }
    }

    if( !(_current_calibration_flags & cv::CALIB_FIX_K1) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          _current_intrinsics.dist_coeffs;

      if( D.size() > 0 && fabs(D[0]) < eps ) {
        _current_calibration_flags |= cv::CALIB_FIX_K1;
      }
    }

    if( !(_current_calibration_flags & cv::CALIB_FIX_K2) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          _current_intrinsics.dist_coeffs;

      if( D.size() > 1 && fabs(D[1]) < eps ) {
        _current_calibration_flags |= cv::CALIB_FIX_K2;
      }
    }

    if( !(_current_calibration_flags & cv::CALIB_FIX_K3) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          _current_intrinsics.dist_coeffs;

      if( D.size() > 4 && fabs(D[4]) < eps ) {
        _current_calibration_flags |= cv::CALIB_FIX_K3;
      }
    }
  }
}

void c_camera_calibration_pipeline::update_undistortion_remap()
{
  cv::initUndistortRectifyMap(_best_intrinsics.camera_matrix,
      _best_intrinsics.dist_coeffs,
      cv::noArray(),
      cv::getOptimalNewCameraMatrix(_best_intrinsics.camera_matrix, _best_intrinsics.dist_coeffs,
          _best_intrinsics.image_size, 0.0, _best_intrinsics.image_size),
      _best_intrinsics.image_size,
      CV_32FC2,
      _current_undistortion_remap,
      cv::noArray());
}


bool c_camera_calibration_pipeline::save_current_camera_parameters() const
{
  if( !_output_intrinsics_filename.empty() ) {

    if( !create_path(get_parent_directory(_output_intrinsics_filename)) ) {
      CF_ERROR("create_path('%s') fails: %s", _output_intrinsics_filename.c_str(),
          strerror(errno));
      return false;
    }

    CF_DEBUG("saving output_intrinsics_filename_: %s", _output_intrinsics_filename.c_str());

    cv::FileStorage fs(_output_intrinsics_filename, cv::FileStorage::WRITE);

    if( !fs.isOpened() ) {
      CF_ERROR("cv::FileStorage('%s') fails: %s ", _output_intrinsics_filename.c_str(),
          strerror(errno));
      return false;
    }

    time_t rawtime;
    time(&rawtime);
    char buf[256];
    strftime(buf, sizeof(buf) - 1, "%c", localtime(&rawtime));

    fs << "calibrationDate" << buf;
    fs << "framesCount" << (int) _object_points.size();
    fs << "calibration_flags" << flagsToString<CAMERA_CALIBRATION_FLAGS>(_current_calibration_flags);

    fs << "cameraResolution" << _current_frame.size();
    fs << "cameraMatrix" << _current_intrinsics.camera_matrix;
    fs << "cameraMatrix_std_dev" << _stdDeviations.rowRange(cv::Range(0, 4));
    fs << "dist_coeffs" << _current_intrinsics.dist_coeffs;
    fs << "dist_coeffs_std_dev" << _stdDeviations.rowRange(cv::Range(4, 9));
    fs << "avg_reprojection_error" << _rmse;

    fs.release();
  }

  return true;
}


//bool c_camera_calibration_pipeline::open_input_sequence()
//{
//  if ( !input_sequence_->open() ) {
//    set_status_msg("ERROR: input_sequence->open() fails");
//    return false;
//  }
//  return true;
//}
//
//void c_camera_calibration_pipeline::close_input_sequence()
//{
//  if ( input_sequence_ ) {
//    input_sequence_->close();
//  }
//}
//
//bool c_camera_calibration_pipeline::seek_input_sequence(int pos)
//{
//  if ( !input_sequence_->seek(pos) ) {
//    CF_ERROR("ERROR: input_sequence->seek(start_pos=%d) fails", pos);
//    return false;
//  }
//  return true;
//}


bool c_camera_calibration_pipeline::write_chessboard_video()
{
  if( !_output_options.save_chessboard_frames ) {
    return true; // nothing to do
  }

  if( _current_frame.empty() ) {
    return true; // wait for next frame
  }

  if( !_chessboard_video_writer.is_open() ) {

    const c_output_frame_writer_options & opts =
        _output_options.output_chessboard_video_options;

    const std::string filename =
        generate_output_filename(opts.output_filename,
            "chessboard",
            ".avi");

    bool fOK =
        _chessboard_video_writer.open(filename,
            opts.ffmpeg_opts,
            opts.output_image_processor,
            opts.output_pixel_depth,
            opts.save_frame_mapping);

    if( !fOK ) {
      CF_ERROR("chessboard_video_writer_.open('%s') fails",
          filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s'", filename.c_str());
  }

  if( !_chessboard_video_writer.write(_current_frame, _current_mask, false, _input_sequence->current_pos() - 1) ) {
    CF_ERROR("chessboard_video_writer_.write() fails");
    return false;
  }

  return true;
}

bool c_camera_calibration_pipeline::write_output_videos()
{
  if ( _output_options.save_coverage_frame ) {

    std::string coverage_frame_filename =
        _output_options.output_coverage_frame_filename.empty() ? "coverage.png" :
            _output_options.output_coverage_frame_filename;

    if ( !is_absolute_path(coverage_frame_filename) ) {
      coverage_frame_filename =
          ssprintf("%s/%s", _output_path.c_str(),
              coverage_frame_filename.c_str());
    }


    cv::Mat3b coverage_image(_current_intrinsics.image_size, cv::Vec3b(0, 0, 0));

    for( const std::vector<cv::Point2f> &corners : _image_points ) {
      for( const cv::Point2f &corner : corners ) {

        cv::rectangle(coverage_image, cv::Rect(corner.x - 1, corner.y - 1, 3, 3),
            CV_RGB(0, 255, 0), -1,
            cv::LINE_8);
      }
    }

    if ( !save_image(coverage_image, coverage_frame_filename) ) {
      CF_ERROR("ERROR: save_image('%s') fails", coverage_frame_filename.c_str());
    }
  }

  if( !_output_options.save_rectified_frames && !_output_options.save_debug_video ) {
    return true;
  }


  CF_DEBUG("update_undistortion_remap()...");
  update_undistortion_remap();

  if( _current_undistortion_remap.empty() ) {
    CF_ERROR("current_undistortion_remap is empty, can not create rectified images");
    return false;
  }

  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }

  CF_DEBUG("Save rectified videos...");

  /////////////////////////////////////////////////////////////////////////////////////////////////

  c_output_frame_writer rectified_viddeo_writer;
  c_output_frame_writer debug_viddeo_writer;

  if ( _output_options.save_rectified_frames ) {

    const c_output_frame_writer_options & opts =
        _output_options.output_rectified_video_options;

    std::string output_filename =
        generate_output_filename(opts.output_filename,
            "rectified",
            ".avi");

    CF_DEBUG("Creating %s...", output_filename.c_str());

    bool fOK =
        rectified_viddeo_writer.open(output_filename,
            opts.ffmpeg_opts,
            opts.output_image_processor,
            opts.output_pixel_depth,
            opts.save_frame_mapping);

    if( !fOK ) {
      CF_ERROR("ERROR: c_video_writer::open('%s') fails", output_filename.c_str());
      return false;
    }

  }

  if ( _output_options.save_debug_video ) {

    const c_output_frame_writer_options & opts =
        _output_options.output_debug_video_options;

    std::string output_filename =
        generate_output_filename(opts.output_filename,
            "debug",
            ".avi");

    CF_DEBUG("Creating %s...", output_filename.c_str());

    bool fOK =
        debug_viddeo_writer.open(output_filename,
            opts.ffmpeg_opts,
            opts.output_image_processor,
            opts.output_pixel_depth,
            opts.save_frame_mapping);

    if( !fOK ) {
      CF_ERROR("ERROR: c_video_writer::open('%s') fails", output_filename.c_str());
      return false;
    }
  }


  _total_frames = _input_sequence->size();
  _processed_frames = 0;
  _accumulated_frames = 0;

  cv::Mat remapped_frame, debug_frame;
  cv::Rect debug_roi[4];

  for( on_status_update(); _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    if( !read_input_frame(_input_sequence, _current_frame, _current_mask) ) {
      set_status_msg("read_input_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    cv::remap(_current_frame, remapped_frame,
        _current_undistortion_remap, cv::noArray(),
        cv::INTER_LINEAR);

      _accumulated_frames =
          _processed_frames;

    if( canceled() ) {
      break;
    }

    if( rectified_viddeo_writer.is_open() ) {

      if( !rectified_viddeo_writer.write(remapped_frame, cv::noArray(), false, _input_sequence->current_pos() - 1) ) {
        CF_ERROR("ERROR: rectified_viddeo_writer.write('%s') fails. Disk full ?", rectified_viddeo_writer.filename().c_str());
        return 1;
      }

      if( canceled() ) {
        break;
      }
    }

    if( debug_viddeo_writer.is_open() ) {

      if ( debug_frame.empty() ) {

        const cv::Size s =
            _current_frame.size();

        const cv::Size display_size(2 * s.width, 2 * s.height);


        debug_roi[0] = cv::Rect(0, 0, s.width, s.height);
        debug_roi[1] = cv::Rect(s.width, 0, s.width, s.height);
        debug_roi[2] = cv::Rect(0, s.height, s.width, s.height);
        debug_roi[3] = cv::Rect(s.width, s.height, s.width, s.height);

        debug_frame.create(display_size, _current_frame.type());
        debug_frame.setTo(cv::Scalar::all(0));
      }

      _current_frame.copyTo(debug_frame(debug_roi[0]));
      remapped_frame.copyTo(debug_frame(debug_roi[1]));
      remapped_frame.copyTo(debug_frame(debug_roi[2]));
      cv::addWeighted(_current_frame, 0.5, remapped_frame, 0.5, 0, debug_frame(debug_roi[3]));

      if( !debug_viddeo_writer.write(debug_frame, cv::noArray(), false, _input_sequence->current_pos() - 1) ) {
        CF_ERROR("ERROR: debug_viddeo_writer.write('%s') fails. Disk full ?", debug_viddeo_writer.filename().c_str());
        return 1;
      }

      if( canceled() ) {
        break;
      }
    }

    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  close_input_sequence();

  return true;
}
