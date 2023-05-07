/*
 * c_stereo_calibration.cc
 *
 *  Created on: Mar 21, 2023
 *      Author: amyznikov
 */

#include "c_stereo_calibration.h"
#include <core/settings/opencv_settings.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

c_stereo_calibration::~c_stereo_calibration()
{
  chessboard_video_writer_.close();
}

c_chessboard_corners_detection_options & c_stereo_calibration::chessboard_detection_options()
{
  return chessboard_detection_options_;
}

const c_chessboard_corners_detection_options & c_stereo_calibration::chessboard_detection_options() const
{
  return chessboard_detection_options_;
}

c_stereo_calibrate_options & c_stereo_calibration::calibration_options()
{
  return calibration_options_;
}

const c_stereo_calibrate_options & c_stereo_calibration::calibration_options() const
{
  return calibration_options_;
}

c_stereo_calibration_output_options & c_stereo_calibration::output_options()
{
  return output_options_;
}

const c_stereo_calibration_output_options & c_stereo_calibration::output_options() const
{
  return output_options_;
}

void c_stereo_calibration::set_output_intrinsics_filename(const std::string & v)
{
  output_intrinsics_filename_ = v;
}

const std::string& c_stereo_calibration::output_intrinsics_filename() const
{
  return output_intrinsics_filename_;
}

void c_stereo_calibration::set_output_extrinsics_filename(const std::string & v)
{
  output_extrinsics_filename_ = v;
}

const std::string & c_stereo_calibration::output_extrinsics_filename() const
{
  return output_extrinsics_filename_;
}

void c_stereo_calibration::set_chessboard_frames_filename(const std::string & v)
{
  chessboard_frames_filename_ = v;
}

const std::string & c_stereo_calibration::chessboard_frames_filename() const
{
  return chessboard_frames_filename_;
}

bool c_stereo_calibration::canceled() const
{
  return false;
}

bool c_stereo_calibration::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

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
    SERIALIZE_OPTION(section, save, calibration_options_, solverTerm);
    SERIALIZE_OPTION(section, save, calibration_options_, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_chessboard_frames);
    SERIALIZE_OPTION(section, save, output_options_, chessboard_frames_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_rectified_frames);
    SERIALIZE_OPTION(section, save, output_options_, rectified_frames_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_stereo_rectified_frames);
    SERIALIZE_OPTION(section, save, output_options_, stereo_rectified_frames_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_quad_rectified_frames);
    SERIALIZE_OPTION(section, save, output_options_, quad_rectified_frames_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
  }

  return true;
}


bool c_stereo_calibration::initialize()
{
  cleanup();

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
          0.f);
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

  return true;
}

void c_stereo_calibration::cleanup()
{
  chessboard_video_writer_.close();

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

//  display_frame_.release();
//  display_mask_.release();
}

bool c_stereo_calibration::update_calibration()
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
          calibration_options_.solverTerm,
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

bool c_stereo_calibration::process_current_stereo_frame(bool enable_calibration)
{
  for( int i = 0; i < 2; ++i ) {
    current_image_points_[i].clear();
  }

  for( int i = 0; i < 2; ++i ) {
    if( !detect_chessboard(current_frames_[i], current_image_points_[i]) ) {
      return true; // wait for next frame
    }
  }

  if ( canceled() ) {
    return false;
  }

  if( output_options_.save_chessboard_frames && !write_horizontal_stereo_video() ) {
    CF_ERROR("write_horizontal_stereo_video() fails");
    return false;
  }


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

  return true;
}

bool c_stereo_calibration::process_stereo_frame(const cv::Mat images[2], const cv::Mat masks[2], bool enable_calibration)
{
  for( int i = 0; i < 2; ++i ) {
    if( images[i].data != current_frames_[i].data ) {
      images[i].copyTo(current_frames_[i]);
    }
    if( masks[i].data != current_masks_[i].data ) {
      masks[i].copyTo(current_masks_[i]);
    }
  }

  return process_current_stereo_frame(enable_calibration);
}


bool c_stereo_calibration::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
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

bool c_stereo_calibration::detect_chessboard(const cv::Mat & frame, std::vector<cv::Point2f> & corners_) const
{
  return find_chessboard_corners(frame,
      chessboard_detection_options_.chessboard_size,
      corners_,
      chessboard_detection_options_);
}

void c_stereo_calibration::estimate_grid_meanstdev(double * m, double * s, int excludedIndex) const
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

  if ( m ) {
    *m = mean[0];
  }
  if ( s ) {
    *s = stdDev[0];
  }
}

double c_stereo_calibration::estimate_coverage_quality(int excludedIndex) const
{
  double mean, stdev;

  estimate_grid_meanstdev(&mean, &stdev,
      excludedIndex);

  return stdev / mean;
}

double c_stereo_calibration::estimate_subset_quality() const
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

void c_stereo_calibration::filter_frames(bool only_landmarks)
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

void c_stereo_calibration::update_state()
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



void c_stereo_calibration::update_undistortion_remap()
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

bool c_stereo_calibration::save_current_camera_parameters() const
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

bool c_stereo_calibration::write_horizontal_stereo_video()
{
  if( chessboard_frames_filename_.empty() ) {
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
        chessboard_video_writer_.open(chessboard_frames_filename_,
            frame.size(),
            frame.channels() > 1,
            false);

    if( !fOK ) {
      CF_ERROR("chessboard_video_writer_.open('%s') fails",
          chessboard_frames_filename_.c_str());
      return false;
    }
  }

  if ( !chessboard_video_writer_.write(frame, cv::noArray(), false, 0) ) {
    CF_ERROR("chessboard_video_writer_.write() fails");
    return false;
  }


  return true;
}
