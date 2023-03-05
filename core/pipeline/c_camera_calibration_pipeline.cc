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

void c_camera_calibration_pipeline::set_chessboard_size(const cv::Size & v)
{
  chessboard_size_ = v;
}

const cv::Size& c_camera_calibration_pipeline::chessboard_size() const
{
  return chessboard_size_;
}

void c_camera_calibration_pipeline::set_chessboard_cell_size(const cv::Size2f & v)
{
  chessboard_cell_size_ = v;
}

const cv::Size2f & c_camera_calibration_pipeline::chessboard_cell_size() const
{
  return chessboard_cell_size_;
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
  return chessboard_corners_detection_options_;
}

const c_chessboard_corners_detection_options & c_camera_calibration_pipeline::chessboard_corners_detection_options() const
{
  return chessboard_corners_detection_options_;
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

bool c_camera_calibration_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  lock_guard lock(accumulator_lock_);
  display_frame_.copyTo(frame);
  mask.release();
  return true;
}

bool c_camera_calibration_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  SERIALIZE_PROPERTY(settings, save, *this, chessboard_size);
  SERIALIZE_PROPERTY(settings, save, *this, chessboard_cell_size);

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, input_options_, start_frame_index);
    SERIALIZE_OPTION(section, save, input_options_, max_input_frames);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "chessboard_corners_detection")) ) {
    SERIALIZE_OBJECT(section, save, chessboard_corners_detection_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "calibration_options")) ) {
    SERIALIZE_OPTION(section, save, calibration_options_, min_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, max_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, calibration_flags);
    SERIALIZE_OPTION(section, save, calibration_options_, auto_tune_calibration_flags);
    SERIALIZE_OPTION(section, save, calibration_options_, solverTerm);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, save_rectified_images);
    SERIALIZE_OPTION(section, save, output_options_, rectified_images_file_name);
  }

  return true;
}

void c_camera_calibration_pipeline::update_output_path()
{
  if( output_directory_.empty() ) {

    std::string parent_directory =
        input_sequence_->sources().empty() ? "." :
            get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/calib",
            parent_directory.c_str());

  }
  else if( !is_absolute_path(output_directory_) ) {

    std::string parent_directory =
        input_sequence_->sources().empty() ? "." :
            get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/%s",
            parent_directory.c_str(),
            output_directory_.c_str());
  }

  if( output_path_.empty() ) {
    output_path_ =
        "./calib";
  }
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


bool c_camera_calibration_pipeline::detect_chessboard(const cv::Mat &frame)
{

  is_chessboard_found_ =
      find_chessboard_corners(frame,
          chessboard_size_,
          current_image_points_,
          chessboard_corners_detection_options_);

//  if( isTemplateFound_ ) {
//    lock_guard lock(accumulator_lock_);
//
//    if( current_frame_.channels() == 1 ) {
//      cv::cvtColor(current_frame_, display_frame_,
//          cv::COLOR_GRAY2BGR);
//    }
//    else {
//      current_frame_.copyTo(display_frame_);
//    }
//
//    cv::drawChessboardCorners(display_frame_,
//        chessboard_size_,
//        current_image_points_,
//        isTemplateFound_);
//  }

  if( !is_chessboard_found_ ) {
    CF_ERROR("find_chessboard_corners() fails");
    return false;
  }

  return is_chessboard_found_;
}

void c_camera_calibration_pipeline::update_undistortion_remap()
{
  cv::initUndistortRectifyMap(intrinsics_.camera_matrix,
      intrinsics_.dist_coeffs,
      cv::noArray(),
      cv::getOptimalNewCameraMatrix(intrinsics_.camera_matrix, intrinsics_.dist_coeffs,
          current_frame_.size(), 0.0, current_frame_.size()),
      current_frame_.size(),
      CV_32FC2,
      current_undistortion_remap_,
      cv::noArray());
}


void c_camera_calibration_pipeline::update_display_image()
{
  if( true ) {

    lock_guard lock(accumulator_lock_);

    accumulated_frames_ =
        image_points_.size();

    if( current_frame_.channels() == 1 ) {
      cv::cvtColor(current_frame_, display_frame_,
          cv::COLOR_GRAY2BGR);
    }
    else {
      current_frame_.copyTo(display_frame_);
    }

    if ( !current_undistortion_remap_.empty() ) {
      cv::remap(display_frame_, display_frame_,
          current_undistortion_remap_, cv::noArray(),
          cv::INTER_LINEAR);
    }
    else {

      cv::drawChessboardCorners(display_frame_,
          chessboard_size_,
          current_image_points_,
          is_chessboard_found_);
    }
  }

  on_accumulator_changed();
}

double c_camera_calibration_pipeline::estimate_grid_subset_quality(size_t excludedIndex) const
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

  cv::Scalar mean, stdDev;
  cv::meanStdDev(pointsInCell, mean, stdDev);

  return mean[0] / (stdDev[0] + 1e-7);
}

double c_camera_calibration_pipeline::estimate_coverage_quality() const
{
  int gridSize = 10;
  int xGridStep = current_frame_.cols / gridSize;
  int yGridStep = current_frame_.rows / gridSize;

  std::vector<int> pointsInCell(gridSize * gridSize);

  std::fill(pointsInCell.begin(), pointsInCell.end(), 0);

  for( auto it = image_points_.begin(); it != image_points_.end(); ++it ) {

    for( auto pointIt = (*it).begin(); pointIt != (*it).end(); ++pointIt ) {
      int i = (int) ((*pointIt).x / xGridStep);
      int j = (int) ((*pointIt).y / yGridStep);
      pointsInCell[i * gridSize + j]++;
    }
  }

  cv::Scalar mean, stdDev;
  cv::meanStdDev(pointsInCell, mean, stdDev);

  return mean[0] / (stdDev[0] + 1e-7);
}

void c_camera_calibration_pipeline::filter_frames()
{
  const int nbframes =
      object_points_.size();

  const int nbframesmax =
      std::max(1, std::max(calibration_options_.min_frames, calibration_options_.max_frames));

  CF_DEBUG("nbframes = %d / nbframesmax = %d", nbframes, nbframesmax);

  if( nbframes != perViewErrors_.rows ) {

    if( nbframes > 1 ) {
      CF_ERROR("APP BUG: nbframes=%d != perViewErrors_.rows=%d",
          nbframes,
          perViewErrors_.rows);
    }

    if( nbframes > nbframesmax ) {
      image_points_.erase(image_points_.begin());
      object_points_.erase(object_points_.begin());
    }

    return;
  }

  if( nbframes > nbframesmax ) {

    double worstValue = -HUGE_VAL;
    double maxQuality = estimate_grid_subset_quality(nbframes);

    size_t worstElemIndex = 0;

    const double alpha =
        calibration_options_.filter_alpha;

    for( size_t i = 0; i < nbframes; i++ ) {

      const double gridQDelta =
          estimate_grid_subset_quality(i) - maxQuality;

      const double currentValue =
          perViewErrors_[i][0] * alpha + gridQDelta * (1. - alpha);

      if( currentValue > worstValue ) {
        worstValue = currentValue;
        worstElemIndex = i;
      }
    }

    image_points_.erase(image_points_.begin() + worstElemIndex);
    object_points_.erase(object_points_.begin() + worstElemIndex);

    cv::Mat1d newErrorsVec(nbframes - 1, 1);

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


void c_camera_calibration_pipeline::update_state()
{
  cv::Matx33d &cameraMatrix =
      intrinsics_.camera_matrix;

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
      if( S.at<double>(4 + i) / fabs(intrinsics_.dist_coeffs[i]) > 1 ) {
        dConfState = false;
      }
    }

    confIntervalsState_ =
        fConfState && cConfState && dConfState;
  }


  if( image_points_.size() > calibration_options_.min_frames ) {
    coverageQualityState_ = estimate_coverage_quality() > 1.8 ? true : false;
  }

  if( calibration_options_.auto_tune_calibration_flags && image_points_.size() > calibration_options_.min_frames ) {

    if( !(calibration_flags_ & cv::CALIB_FIX_ASPECT_RATIO) ) {

      const double fDiff =
          fabs(cameraMatrix(0, 0) - cameraMatrix(1, 1));

      const cv::Mat &S =
          stdDeviations_;

      if( fDiff < 3 * S.at<double>(0) && fDiff < 3 * S.at<double>(1) ) {
        calibration_flags_ |= cv::CALIB_FIX_ASPECT_RATIO;
        cameraMatrix(0, 0) = cameraMatrix(1, 1);
      }
    }

    if( !(calibration_flags_ & cv::CALIB_ZERO_TANGENT_DIST) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          intrinsics_.dist_coeffs;

      if( D.size() > 3 && fabs(D[2]) < eps && fabs(D[3]) < eps ) {
        calibration_flags_ |= cv::CALIB_ZERO_TANGENT_DIST;
      }
    }

    if( !(calibration_flags_ & cv::CALIB_FIX_K1) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          intrinsics_.dist_coeffs;

      if( D.size() > 0 && fabs(D[0]) < eps ) {
        calibration_flags_ |= cv::CALIB_FIX_K1;
      }
    }

    if( !(calibration_flags_ & cv::CALIB_FIX_K2) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          intrinsics_.dist_coeffs;

      if( D.size() > 1 && fabs(D[1]) < eps ) {
        calibration_flags_ |= cv::CALIB_FIX_K2;
      }
    }

    if( !(calibration_flags_ & cv::CALIB_FIX_K3) ) {

      const double eps = 0.005;

      const std::vector<double> &D =
          intrinsics_.dist_coeffs;

      if( D.size() > 4 && fabs(D[4]) < eps ) {
        calibration_flags_ |= cv::CALIB_FIX_K3;
      }
    }
  }
}


bool c_camera_calibration_pipeline::save_current_camera_parameters() const
{
  if( !create_path(output_path_) ) {
    CF_ERROR("create_path('%s') fails: %s ", output_path_.c_str(), strerror(errno));
    return false;
  }

  std::string output_file_name =
      ssprintf("%s/calib.%s.yml",
          output_path_.c_str(),
          csequence_name());

  cv::FileStorage parametersWriter(output_file_name, cv::FileStorage::WRITE);

  if( !parametersWriter.isOpened() ) {
    CF_ERROR("cv::FileStorage('%s') fails: %s ", output_file_name.c_str(), strerror(errno));
    return false;
  }

  time_t rawtime;
  time(&rawtime);
  char buf[256];
  strftime(buf, sizeof(buf) - 1, "%c", localtime(&rawtime));

  parametersWriter << "calibrationDate" << buf;
  parametersWriter << "framesCount" << (int) object_points_.size();
  parametersWriter << "calibration_flags" << flagsToString<CAMERA_CALIBRATION_FLAGS>(calibration_flags_);

  parametersWriter << "cameraResolution" << current_frame_.size();
  parametersWriter << "cameraMatrix" << intrinsics_.camera_matrix;
  parametersWriter << "cameraMatrix_std_dev" << stdDeviations_.rowRange(cv::Range(0, 4));
  parametersWriter << "dist_coeffs" << intrinsics_.dist_coeffs;
  parametersWriter << "dist_coeffs_std_dev" << stdDeviations_.rowRange(cv::Range(4, 9));
  parametersWriter << "avg_reprojection_error" << rmse_;

  parametersWriter.release();

  return true;
}


bool c_camera_calibration_pipeline::initialize_pipeline()
{
  set_pipeline_stage(camera_calibration_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_chessboard_camera_calibration_pipeline: base::initialize() fails");
    return false;
  }

  is_chessboard_found_ = false;
  current_image_points_.clear();
  current_object_points_.clear();

  image_points_.clear();
  object_points_.clear();

  intrinsics_.camera_matrix = cv::Matx33d::zeros();
  intrinsics_.dist_coeffs.clear();
  stdDeviations_.release();
  perViewErrors_.release();
  current_undistortion_remap_.release();

  calibration_flags_ = calibration_options_.calibration_flags;
  confIntervalsState_ = false;
  coverageQualityState_ = false;
  intrinsics_initialized_ = false;

  if ( chessboard_size_.width < 2 || chessboard_size_.height < 2 ) {
    CF_ERROR("Invalid chessboard_size_: %dx%d", chessboard_size_.width, chessboard_size_.height);
    return false;
  }

  if ( !(chessboard_cell_size_.width > 0) || !(chessboard_cell_size_.height > 0)  ) {
    CF_ERROR("Invalid chessboard_cell_size_: %gx%g", chessboard_cell_size_.width, chessboard_cell_size_.height);
    return false;
  }

  current_object_points_.clear();
  current_object_points_.reserve(chessboard_size_.area());

  for( int i = 0; i < chessboard_size_.height; ++i ) {
    for( int j = 0; j < chessboard_size_.width; ++j ) {

      current_object_points_.emplace_back(
          j * chessboard_cell_size_.width,
          i * chessboard_cell_size_.height,
          0.f);
    }
  }

  return true;
}

void c_camera_calibration_pipeline::cleanup_pipeline()
{
  set_pipeline_stage(camera_calibration_finishing);

  base::cleanup_pipeline();

  current_image_points_.clear();
  current_object_points_.clear();

  image_points_.clear();
  object_points_.clear();

  stdDeviations_.release();
  perViewErrors_.release();
  current_undistortion_remap_.release();

  current_frame_.release();
  current_mask_.release();


  set_pipeline_stage(camera_calibration_idle);
}

bool c_camera_calibration_pipeline::run_pipeline()
{
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

    if ( canceled() ) {
      break;
    }

    if ( is_bad_frame_index(input_sequence_->current_pos())) {
      CF_DEBUG("Skip frame %d as blacklisted", input_sequence_->current_pos());
      input_sequence_->seek(input_sequence_->current_pos() + 1);
      continue;
    }

    filter_frames();

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

    if( !detect_chessboard(current_frame_) ) {
      CF_ERROR("detect_chessboard() fails");
      continue;
    }

    image_points_.emplace_back(current_image_points_);
    object_points_.emplace_back(current_object_points_);

    if ( image_points_.size() >= calibration_options_.min_frames ) {

      const cv::Size image_size =
          current_frame_.size();

      if( !intrinsics_initialized_ ) {

        if( !(calibration_options_.calibration_flags & CAMERA_CALIB_USE_INTRINSIC_GUESS) ) {

          intrinsics_.image_size = image_size;
          intrinsics_.camera_matrix = cv::Matx33d::zeros();
          intrinsics_.dist_coeffs.clear();
          intrinsics_initialized_ = true;
        }

        else {

          intrinsics_initialized_ =
              init_camera_intrinsics(intrinsics_,
                  object_points_,
                  image_points_,
                  image_size);

          if( !intrinsics_initialized_ ) {
            CF_ERROR("init_camera_intrinsics() fails");
            continue;
          }
        }
      }


      rmse_ =
          calibrate_camera(object_points_,
              image_points_,
              intrinsics_,
              calibration_flags_,
              calibration_options_.solverTerm,
              nullptr,
              nullptr,
              &stdDeviations_,
              nullptr,
              &perViewErrors_);

      const cv::Matx33d & M =
          intrinsics_.camera_matrix;

      CF_DEBUG("calibrateCamera:\n"
          "rmse = %g\n"
          "M: {\n"
          "  %+g %+g %+g\n"
          "  %+g %+g %+g\n"
          "  %+g %+g %+g\n"
          "}\n",
          rmse_,
          M(0,0), M(0,1), M(0,2),
          M(1,0), M(1,1), M(1,2),
          M(2,0), M(2,1), M(2,2));

      if ( canceled() ) {
        break;
      }

      update_state();

      if ( canceled() ) {
        break;
      }

      if ( !save_current_camera_parameters() ) {
        CF_ERROR("save_current_camera_parameters() fails");
        return false;
      }

      update_undistortion_remap();

      if ( canceled() ) {
        break;
      }

      update_display_image();
    }

    if ( canceled() ) {
      break;
    }
  }

  input_sequence_->close();

  /////////////////////////////////////////////////////////////////////////////////////////////////

  if( !canceled() && output_options_.save_rectified_images ) {

    if( current_undistortion_remap_.empty() ) {
      CF_ERROR("current_undistortion_remap is empty, can not create rectified images");
    }
    else {

      c_video_writer writer;

      std::string output_file_name =
          output_options_.rectified_images_file_name;

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
