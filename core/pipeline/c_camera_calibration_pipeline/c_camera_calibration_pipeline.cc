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
  return input_options_;
}

const c_camera_calibration_input_options & c_camera_calibration_pipeline::input_options() const
{
  return input_options_;
}

c_chessboard_corners_detection_options & c_camera_calibration_pipeline::chessboard_corners_detection_options()
{
  return chessboard_detection_options_;
}

const c_chessboard_corners_detection_options & c_camera_calibration_pipeline::chessboard_corners_detection_options() const
{
  return chessboard_detection_options_;
}

c_calibrate_camera_options & c_camera_calibration_pipeline::calibrate_camera_options()
{
  return calibration_options_;
}

const c_calibrate_camera_options & c_camera_calibration_pipeline::calibrate_camera_options() const
{
  return calibration_options_;
}

c_camera_calibration_output_options& c_camera_calibration_pipeline::output_options()
{
  return output_options_;
}

const c_camera_calibration_output_options& c_camera_calibration_pipeline::output_options() const
{
  return output_options_;
}

bool c_camera_calibration_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if ( current_frame_.empty() ) {
    return false;
  }

  const cv::Size totalSize(current_frame_.cols * 2,
      current_frame_.rows);

  const cv::Rect roi[2] = {
      cv::Rect(0, 0, current_frame_.cols, current_frame_.rows),
      cv::Rect(current_frame_.cols, 0, current_frame_.cols, current_frame_.rows),
  };

  display_frame.create(totalSize,
      CV_MAKETYPE(current_frame_.depth(), 3));

  cv::Mat & display_frame_ =
      display_frame.getMatRef();

  if( current_frame_.channels() == 3 ) {
    current_frame_.copyTo(display_frame_(roi[0]));
  }
  else {
    cv::cvtColor(current_frame_, display_frame_(roi[0]),
        cv::COLOR_GRAY2BGR);
  }

  if ( is_chessboard_found_ ) {

    cv::drawChessboardCorners(display_frame_(roi[0]),
        chessboard_detection_options_.chessboard_size,
        current_image_points_,
        is_chessboard_found_);
  }

  if( !current_undistortion_remap_.empty() ) {

    if( display_frame_.channels() == current_frame_.channels() ) {

        cv::remap(current_frame_, display_frame_(roi[1]),
            current_undistortion_remap_, cv::noArray(),
            cv::INTER_LINEAR);
    }
    else {
      cv::Mat tmp;

      cv::remap(current_frame_, tmp,
          current_undistortion_remap_, cv::noArray(),
          cv::INTER_LINEAR);

      cv::cvtColor(tmp, display_frame_(roi[1]),
          cv::COLOR_GRAY2BGR);
    }

  }
  else {

    // draw coverage

    if( current_frame_.channels() == 3 ) {
      current_frame_.copyTo(display_frame_(roi[1]));
    }
    else {
      cv::cvtColor(current_frame_, display_frame_(roi[1]),
          cv::COLOR_GRAY2BGR);
    }

    cv::Mat3b display =
        display_frame_(roi[1]);

    for( const std::vector<cv::Point2f> &corners : image_points_ ) {
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
    serialize_base_input_options(section, save, input_options_);
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
    SERIALIZE_OPTION(section, save, calibration_options_, max_iterations);
    SERIALIZE_OPTION(section, save, calibration_options_, solver_eps);
    SERIALIZE_OPTION(section, save, calibration_options_, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_chessboard_frames);
    if ( (subsection = SERIALIZE_GROUP(section, save, "output_chessboard_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, output_chessboard_video_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, save_rectified_frames);
    if ( (subsection = SERIALIZE_GROUP(section, save, "output_rectified_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, output_rectified_video_options);
    }

//    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
//    if ( (subsection = SERIALIZE_GROUP(section, save, "output_chessboard_video_options")) ) {
//      SERIALIZE_OPTION(subsection, save, output_options_, output_progress_video_options);
//    }

  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl> & c_camera_calibration_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Chessboard corners detection", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.method, "Method", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.chessboard_size, "chessboard_size", "");
      PIPELINE_CTL(ctrls, chessboard_detection_options_.chessboard_cell_size, "chessboard_cell_size", "");

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

    PIPELINE_CTL_GROUP(ctrls, "Calibration options", "");
    PIPELINE_CTL(ctrls,  calibration_options_.enable_calibration, "enable_calibration", "");
    PIPELINE_CTL(ctrls,  calibration_options_.min_frames, "min_frames", "");
    PIPELINE_CTL(ctrls,  calibration_options_.max_frames, "max_frames", "");
    PIPELINE_CTL_BITFLAGS(ctrls, calibration_options_.calibration_flags, CAMERA_CALIBRATION_FLAGS,  "calibration flags", "" );
    PIPELINE_CTL(ctrls,  calibration_options_.auto_tune_calibration_flags, "auto_tune_calibration_flags", "");
    PIPELINE_CTL(ctrls,  calibration_options_.init_camera_matrix_2d, "init_camera_matrix_2d", "");
    PIPELINE_CTL(ctrls,  calibration_options_.max_iterations, "max_iterations", "");
    PIPELINE_CTL(ctrls,  calibration_options_.solver_eps, "solver_eps", "");
    PIPELINE_CTL(ctrls,  calibration_options_.filter_alpha, "filter_alpha", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");
      PIPELINE_CTL(ctrls, output_options_.default_display_type, "display_type", "");
      PIPELINE_CTL(ctrls, output_options_.output_intrinsics_filename, "intrinsics_filename", "");

      PIPELINE_CTL_GROUP(ctrls, "Save Chessboard Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_chessboard_frames, "save_chessboard_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.output_chessboard_video_options,
            _this->output_options_.save_chessboard_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "Save Rectified Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_chessboard_frames, "save_rectified_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.output_rectified_video_options,
            _this->output_options_.save_rectified_frames);
      PIPELINE_CTL_END_GROUP(ctrls);

//      PIPELINE_CTL_GROUP(ctrls, "Save Progress Frames", "");
//        PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
//        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.output_progress_video_options,
//            _this->output_options_.save_progress_video);
//      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
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

bool c_camera_calibration_pipeline::initialize_pipeline()
{
   if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
    return false;
  }

  output_path_ =
      create_output_path(output_options_.output_directory);

  output_intrinsics_filename_ =
      generate_output_filename(output_options_.output_intrinsics_filename,
          "camera_intrinsics",
          ".yml");

  chessboard_video_writer_.close();
  current_image_points_.clear();
  current_object_points_.clear();
  stdDeviations_.release();
  perViewErrors_.release();
  current_undistortion_remap_.release();
  image_points_.clear();
  object_points_.clear();
  intrinsics_initialized_ = false;
  is_chessboard_found_ = false;
  confIntervalsState_ = false;

  current_calibration_flags_ = calibration_options_.calibration_flags;
  best_subset_quality_ = HUGE_VAL;

  current_intrinsics_.camera_matrix = cv::Matx33d::zeros();
  current_intrinsics_.dist_coeffs.clear();

  if ( chessboard_detection_options_.chessboard_size.width < 2 || chessboard_detection_options_.chessboard_size.height < 2 ) {
    CF_ERROR("Invalid chessboard_size_: %dx%d", chessboard_detection_options_.chessboard_size.width,
        chessboard_detection_options_.chessboard_size.height);
    return false;
  }

  if ( !(chessboard_detection_options_.chessboard_cell_size.width > 0) || !(chessboard_detection_options_.chessboard_cell_size.height > 0)  ) {
    CF_ERROR("Invalid chessboard_cell_size_: %gx%g", chessboard_detection_options_.chessboard_cell_size.width,
        chessboard_detection_options_.chessboard_cell_size.height);
    return false;
  }

  current_object_points_.reserve(chessboard_detection_options_.chessboard_size.area());

  for( int i = 0; i < chessboard_detection_options_.chessboard_size.height; ++i ) {
    for( int j = 0; j < chessboard_detection_options_.chessboard_size.width; ++j ) {

      current_object_points_.emplace_back(
          j * chessboard_detection_options_.chessboard_cell_size.width,
          i * chessboard_detection_options_.chessboard_cell_size.height,
          0.f);
    }
  }

  return true;
}

void c_camera_calibration_pipeline::cleanup_pipeline()
{
  close_input_sequence();

  if( chessboard_video_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", chessboard_video_writer_.filename().c_str());
    chessboard_video_writer_.close();
  }

  current_image_points_.clear();
  current_object_points_.clear();
  stdDeviations_.release();
  perViewErrors_.release();
  current_undistortion_remap_.release();
  image_points_.clear();
  object_points_.clear();
  intrinsics_initialized_ = false;
  is_chessboard_found_ = false;
  confIntervalsState_ = false;

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

  if ( input_sequence_->is_live() ) {

    total_frames_ = INT_MAX;
    processed_frames_ = 0;
    accumulated_frames_ = 0;

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
  }


  set_status_msg("RUNNING ...");

  const bool is_live_sequence =
      input_sequence_->is_live();

  const bool enable_live_calibration =
      is_live_sequence &&
          calibration_options_.enable_calibration;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( !is_live_sequence && is_bad_frame_index(input_sequence_->current_pos()) ) {
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

    if( input_options_.input_image_processor ) {
      lock_guard lock(mutex());
      if( !input_options_.input_image_processor->process(current_frame_, current_mask_) ) {
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

    accumulated_frames_ =
          object_points_.size();


    // give chance GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if ( canceled() ) {
      break;
    }
  }

  close_input_sequence();


  if ( !is_live_sequence && calibration_options_.enable_calibration ) {

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
  if( current_frame_.empty() || !detect_chessboard(current_frame_) ) {
    return true; // wait for next frame
  }

  if( !write_chessboard_video() ) {
    CF_ERROR("write_chessboard_video() fails");
    return false;
  }

  lock_guard lock(mutex());

  image_points_.emplace_back(current_image_points_);
  object_points_.emplace_back(current_object_points_);

  if( image_points_.size() >= calibration_options_.min_frames ) {

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
  is_chessboard_found_ =
      find_chessboard_corners(frame,
          chessboard_detection_options_.chessboard_size,
          current_image_points_,
          chessboard_detection_options_);

  return is_chessboard_found_;
}

void c_camera_calibration_pipeline::filter_frames(bool only_landmarks)
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
            sum += perViewErrors[i][0];
          }
          return sum;
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
            (totalRmeQuality - perViewErrors_[i][0]) / (perViewErrors_.rows - 1);

        currentSubsetQuality =
            alpha * currentRmseQuality + (1. - alpha) * currentCoverageQuality;
      }

      if( currentSubsetQuality < bestSubsetQuality ) {
        bestSubsetQuality = currentSubsetQuality;
        worstElemIndex = i;
      }
    }

    // CF_DEBUG("worstElemIndex=%d bestSubsetQuality=%g", worstElemIndex, bestSubsetQuality);

    image_points_.erase(image_points_.begin() + worstElemIndex);
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

double c_camera_calibration_pipeline::estimate_subset_quality() const
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
  int xGridStep = current_frame_.cols / gridSize;
  int yGridStep = current_frame_.rows / gridSize;

  std::vector<int> pointsInCell(gridSize * gridSize);

  std::fill(pointsInCell.begin(), pointsInCell.end(), 0);

  for( size_t k = 0; k < image_points_.size(); k++ )
    if( k != excludedIndex ) {

      for( auto pointIt = image_points_[k].begin(); pointIt != image_points_[k].end(); ++pointIt ) {

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
      current_frame_.size();

  if( !intrinsics_initialized_ ) {

    if( !calibration_options_.init_camera_matrix_2d ) {

      current_intrinsics_.image_size = image_size;
      current_intrinsics_.camera_matrix = cv::Matx33d::zeros();
      current_intrinsics_.dist_coeffs.clear();
    }

    else {

      intrinsics_initialized_ =
          init_camera_intrinsics(current_intrinsics_,
              object_points_,
              image_points_,
              image_size);

      if( !intrinsics_initialized_ ) {
        CF_ERROR("init_camera_intrinsics() fails");
        // update_display_image();
        return true; // wait for next frame
      }

      if( canceled() ) {
        return false;
      }
    }

    best_intrinsics_ = current_intrinsics_;
    best_calibration_flags_ = current_calibration_flags_;
  }

  if ( best_subset_quality_ < HUGE_VAL ) {
    current_intrinsics_ = best_intrinsics_;
    current_calibration_flags_ = best_calibration_flags_;
  }

  rmse_ =
      calibrate_camera(object_points_,
          image_points_,
          current_intrinsics_,
          current_calibration_flags_,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
              calibration_options_.max_iterations,
              calibration_options_.solver_eps),
          nullptr,
          nullptr,
          &stdDeviations_,
          nullptr,
          &perViewErrors_);

  const cv::Matx33d & M =
      current_intrinsics_.camera_matrix;

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
      rmse_);

  if ( canceled() ) {
    return false;
  }

  if( rmse_ >= 0 ) {

    intrinsics_initialized_ = true;

    const double subset_quality =
        estimate_subset_quality();

    CF_DEBUG("subset_quality=%g best_subset_quality_=%g",
        subset_quality, best_subset_quality_);

    if( subset_quality < best_subset_quality_ ) {

      best_intrinsics_ = current_intrinsics_;
      best_calibration_flags_ = current_calibration_flags_;
      best_subset_quality_ = subset_quality;

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
      current_intrinsics_.camera_matrix;

  if( true) {

    const double relErrEps = 0.05;
    static const double sigmaMult = 1.96;

    bool fConfState = false;
    bool cConfState = false;
    bool dConfState = true;

    const cv::Mat &S =
        stdDeviations_;

    if( sigmaMult * S.at<double>(0) / cameraMatrix(0, 0) < relErrEps &&
        sigmaMult * S.at<double>(1) / cameraMatrix(1, 1) < relErrEps ) {
      fConfState = true;
    }

    if( sigmaMult * S.at<double>(2) / cameraMatrix(0, 2) < relErrEps &&
        sigmaMult * S.at<double>(3) / cameraMatrix(1, 2) < relErrEps ) {
      cConfState = true;
    }

    for( int i = 0; i < 5; i++ ) {
      if( S.at<double>(4 + i) / fabs(current_intrinsics_.dist_coeffs[i]) > 1 ) {
        dConfState = false;
      }
    }

    confIntervalsState_ =
        fConfState && cConfState && dConfState;
  }

  if( calibration_options_.auto_tune_calibration_flags && image_points_.size() > calibration_options_.min_frames ) {

    if( !(current_calibration_flags_ & cv::CALIB_FIX_ASPECT_RATIO) ) {

      const double fDiff =
          fabs(cameraMatrix(0, 0) - cameraMatrix(1, 1));

      const cv::Mat &S =
          stdDeviations_;

      if( fDiff < 3 * S.at<double>(0) && fDiff < 3 * S.at<double>(1) ) {
        current_calibration_flags_ |= cv::CALIB_FIX_ASPECT_RATIO;
        cameraMatrix(0, 0) = cameraMatrix(1, 1);
      }
    }

    if( !(current_calibration_flags_ & cv::CALIB_ZERO_TANGENT_DIST) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          current_intrinsics_.dist_coeffs;

      if( D.size() > 3 && fabs(D[2]) < eps && fabs(D[3]) < eps ) {
        current_calibration_flags_ |= cv::CALIB_ZERO_TANGENT_DIST;
      }
    }

    if( !(current_calibration_flags_ & cv::CALIB_FIX_K1) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          current_intrinsics_.dist_coeffs;

      if( D.size() > 0 && fabs(D[0]) < eps ) {
        current_calibration_flags_ |= cv::CALIB_FIX_K1;
      }
    }

    if( !(current_calibration_flags_ & cv::CALIB_FIX_K2) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          current_intrinsics_.dist_coeffs;

      if( D.size() > 1 && fabs(D[1]) < eps ) {
        current_calibration_flags_ |= cv::CALIB_FIX_K2;
      }
    }

    if( !(current_calibration_flags_ & cv::CALIB_FIX_K3) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          current_intrinsics_.dist_coeffs;

      if( D.size() > 4 && fabs(D[4]) < eps ) {
        current_calibration_flags_ |= cv::CALIB_FIX_K3;
      }
    }
  }
}

void c_camera_calibration_pipeline::update_undistortion_remap()
{
  cv::initUndistortRectifyMap(best_intrinsics_.camera_matrix,
      best_intrinsics_.dist_coeffs,
      cv::noArray(),
      cv::getOptimalNewCameraMatrix(best_intrinsics_.camera_matrix, best_intrinsics_.dist_coeffs,
          best_intrinsics_.image_size, 0.0, best_intrinsics_.image_size),
      best_intrinsics_.image_size,
      CV_32FC2,
      current_undistortion_remap_,
      cv::noArray());
}


bool c_camera_calibration_pipeline::save_current_camera_parameters() const
{
  if( !output_intrinsics_filename_.empty() ) {

    if( !create_path(get_parent_directory(output_intrinsics_filename_)) ) {
      CF_ERROR("create_path('%s') fails: %s", output_intrinsics_filename_.c_str(),
          strerror(errno));
      return false;
    }

    CF_DEBUG("saving output_intrinsics_filename_: %s", output_intrinsics_filename_.c_str());

    cv::FileStorage fs(output_intrinsics_filename_, cv::FileStorage::WRITE);

    if( !fs.isOpened() ) {
      CF_ERROR("cv::FileStorage('%s') fails: %s ", output_intrinsics_filename_.c_str(),
          strerror(errno));
      return false;
    }

    time_t rawtime;
    time(&rawtime);
    char buf[256];
    strftime(buf, sizeof(buf) - 1, "%c", localtime(&rawtime));

    fs << "calibrationDate" << buf;
    fs << "framesCount" << (int) object_points_.size();
    fs << "calibration_flags" << flagsToString<CAMERA_CALIBRATION_FLAGS>(current_calibration_flags_);

    fs << "cameraResolution" << current_frame_.size();
    fs << "cameraMatrix" << current_intrinsics_.camera_matrix;
    fs << "cameraMatrix_std_dev" << stdDeviations_.rowRange(cv::Range(0, 4));
    fs << "dist_coeffs" << current_intrinsics_.dist_coeffs;
    fs << "dist_coeffs_std_dev" << stdDeviations_.rowRange(cv::Range(4, 9));
    fs << "avg_reprojection_error" << rmse_;

    fs.release();
  }

  return true;
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
  if ( input_sequence_ ) {
    input_sequence_->close();
  }
}

bool c_camera_calibration_pipeline::seek_input_sequence(int pos)
{
  if ( !input_sequence_->seek(pos) ) {
    CF_ERROR("ERROR: input_sequence->seek(start_pos=%d) fails", pos);
    return false;
  }
  return true;
}


bool c_camera_calibration_pipeline::write_chessboard_video()
{
  if( !output_options_.save_chessboard_frames ) {
    return true; // nothing to do
  }

  if( current_frame_.empty() ) {
    return true; // wait for next frame
  }

  if( !chessboard_video_writer_.is_open() ) {

    const c_output_frame_writer_options & opts =
        output_options_.output_chessboard_video_options;

    const std::string filename =
        generate_output_filename(opts.output_filename,
            "chessboard",
            ".avi");

    bool fOK =
        chessboard_video_writer_.open(filename,
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

  if( !chessboard_video_writer_.write(current_frame_, current_mask_, false, input_sequence_->current_pos() - 1) ) {
    CF_ERROR("chessboard_video_writer_.write() fails");
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

  const c_output_frame_writer_options & opts =
      output_options_.output_rectified_video_options;

  std::string output_filename =
      generate_output_filename(opts.output_filename,
          "rectified",
          ".avi");


  CF_DEBUG("Saving %s...", output_filename.c_str());


  bool fOK =
      writer.open(output_filename,
          opts.ffmpeg_opts,
          opts.output_image_processor,
          opts.output_pixel_depth,
          opts.save_frame_mapping);

  if( !fOK ) {
    CF_ERROR("ERROR: c_video_writer::open('%s') fails", output_filename.c_str());
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
