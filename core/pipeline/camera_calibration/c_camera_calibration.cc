/*
 * c_camera_calibration.cc
 *
 *  Created on: Mar 25, 2023
 *      Author: amyznikov
 */

#include "c_camera_calibration.h"
#include <core/settings/opencv_settings.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

c_chessboard_corners_detection_options & c_camera_calibration::chessboard_corners_detection_options()
{
  return chessboard_detection_options_;
}

const c_chessboard_corners_detection_options & c_camera_calibration::chessboard_corners_detection_options() const
{
  return chessboard_detection_options_;
}

c_calibrate_camera_options & c_camera_calibration::calibrate_camera_options()
{
  return calibration_options_;
}

const c_calibrate_camera_options & c_camera_calibration::calibrate_camera_options() const
{
  return calibration_options_;
}

c_camera_calibration_output_options& c_camera_calibration::output_options()
{
  return output_options_;
}

const c_camera_calibration_output_options& c_camera_calibration::output_options() const
{
  return output_options_;
}

void c_camera_calibration::set_output_intrinsics_filename(const std::string & v)
{
  output_intrinsics_filename_ = v;
}

const std::string& c_camera_calibration::output_intrinsics_filename() const
{
  return output_intrinsics_filename_;
}

bool c_camera_calibration::canceled() const
{
  return false;
}

bool c_camera_calibration::detect_chessboard(const cv::Mat &frame)
{
  is_chessboard_found_ =
      find_chessboard_corners(frame,
          chessboard_detection_options_.chessboard_size,
          current_image_points_,
          chessboard_detection_options_);

  if( !is_chessboard_found_ ) {
    // CF_ERROR("find_chessboard_corners() fails");
    return false;
  }

  return is_chessboard_found_;
}


void c_camera_calibration::filter_frames()
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


void c_camera_calibration::update_state()
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


  if( image_points_.size() > calibration_options_.min_frames ) {
    coverageQualityState_ = estimate_coverage_quality() > 1.8 ? true : false;
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

void c_camera_calibration::estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const
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

double c_camera_calibration::estimate_grid_subset_quality(size_t excludedIndex) const
{
  double m, s;
  estimate_grid_meanstdev(&m, &s, excludedIndex);
  return m / (s + 1e-7);
}

double c_camera_calibration::estimate_coverage_quality() const
{
  double m, s;
  estimate_grid_meanstdev(&m, &s, object_points_.size());
  return m / (s + 1e-7);
}


double c_camera_calibration::estimate_subset_quality() const
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


void c_camera_calibration::update_undistortion_remap()
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

bool c_camera_calibration::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  if ( display_frame.needed() ) {
    display_frame_.copyTo(display_frame);
  }
  if ( display_mask.needed() ) {
    display_mask_.copyTo(display_mask);
  }
  return true;
}

void c_camera_calibration::update_display_image()
{
  CF_DEBUG("c_camera_calibration::update_display_image()");

  const cv::Size totalSize(current_frame_.cols * 2, current_frame_.rows);

  const cv::Rect roi[2] = {
      cv::Rect(0, 0, current_frame_.cols, current_frame_.rows),
      cv::Rect(current_frame_.cols, 0, current_frame_.cols, current_frame_.rows),
  };

  display_frame_.create(totalSize, CV_MAKETYPE(current_frame_.depth(), 3));

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

  if( current_undistortion_remap_.empty() ) {

    if( current_frame_.channels() == 3 ) {
      current_frame_.copyTo(display_frame_(roi[1]));
    }
    else {
      cv::cvtColor(current_frame_, display_frame_(roi[1]),
          cv::COLOR_GRAY2BGR);
    }

  }
  else if( display_frame_.channels() == current_frame_.channels() ) {

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


bool c_camera_calibration::save_current_camera_parameters() const
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

bool c_camera_calibration::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if( (section = SERIALIZE_GROUP(settings, save, "chessboard_detection")) ) {
    SERIALIZE_OBJECT(section, save, chessboard_detection_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "calibration_options")) ) {
    SERIALIZE_OPTION(section, save, calibration_options_, min_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, max_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, calibration_flags);
    SERIALIZE_OPTION(section, save, calibration_options_, auto_tune_calibration_flags);
    SERIALIZE_OPTION(section, save, calibration_options_, solverTerm);
    SERIALIZE_OPTION(section, save, calibration_options_, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_rectified_frames);
    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);

    SERIALIZE_OPTION(section, save, output_options_, rectified_frames_filename);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
  }

  return true;
}

bool c_camera_calibration::initialize()
{
  current_image_points_.clear();
  current_object_points_.clear();
  image_points_.clear();
  object_points_.clear();

  is_chessboard_found_ = false;
  intrinsics_initialized_ = false;
  confIntervalsState_ = false;
  coverageQualityState_ = false;
  current_calibration_flags_ = calibration_options_.calibration_flags;
  best_subset_quality_ = HUGE_VAL;

  current_intrinsics_.camera_matrix = cv::Matx33d::zeros();
  current_intrinsics_.dist_coeffs.clear();
  stdDeviations_.release();
  perViewErrors_.release();
  current_undistortion_remap_.release();

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

  current_object_points_.clear();
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

void c_camera_calibration::cleanup()
{
  current_image_points_.clear();
  current_object_points_.clear();

  image_points_.clear();
  object_points_.clear();

  stdDeviations_.release();
  perViewErrors_.release();
  current_undistortion_remap_.release();

  current_frame_.release();
  current_mask_.release();
}

bool c_camera_calibration::process_frame(const cv::Mat & image, const cv::Mat & mask)
{
  if( &image != &current_frame_ ) {
    image.copyTo(current_frame_);
  }

  if( &mask != &current_mask_ ) {
    mask.copyTo(current_mask_);
  }

  if( !detect_chessboard(current_frame_) ) {
    update_display_image();
    return true; // wait for next frame
  }

  filter_frames();

  if ( canceled() ) {
    return false;
  }

  image_points_.emplace_back(current_image_points_);
  object_points_.emplace_back(current_object_points_);

  if ( image_points_.size() >= calibration_options_.min_frames ) {

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
          update_display_image();
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
      CF_DEBUG("USE best* as guess");
      current_intrinsics_ = best_intrinsics_;
      current_calibration_flags_ = best_calibration_flags_;
    }

    rmse_ =
        calibrate_camera(object_points_,
            image_points_,
            current_intrinsics_,
            current_calibration_flags_,
            calibration_options_.solverTerm,
            nullptr,
            nullptr,
            &stdDeviations_,
            nullptr,
            &perViewErrors_);

    const cv::Matx33d & M =
        current_intrinsics_.camera_matrix;

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
      return false;
    }

    if( rmse_ >= 0 ) {

      intrinsics_initialized_ = true;

      const double subset_quality =
          estimate_subset_quality();

      CF_DEBUG("subset_quality=%g best_subset_quality_=%g", subset_quality, best_subset_quality_);

      if( subset_quality < best_subset_quality_ ) {
        CF_DEBUG("Copy to best*");

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
  }

  if ( canceled() ) {
    return false;
  }

  update_display_image();

  return true;
}

