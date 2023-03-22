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

template<>
const c_enum_member* members_of<STEREO_CALIBRATION_STAGE>()
{
  static constexpr c_enum_member members[] = {
      { stereo_calibration_idle, "idle", "" },
      { stereo_calibration_initialize, "initialize", "" },
      { stereo_calibration_in_progress, "in_progress", "" },
      { stereo_calibration_finishing, "finishing", "" },
      { stereo_calibration_idle }
  };

  return members;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_chessboard_corners_detection_options & c_stereo_calibration::chessboard_detection_options()
{
  return chessboard_detection_options_;
}

const c_chessboard_corners_detection_options & c_stereo_calibration::chessboard_detection_options() const
{
  return chessboard_detection_options_;
}

c_stereo_calibrate_options & c_stereo_calibration::stereo_calibrate_options()
{
  return calibration_options_;
}

const c_stereo_calibrate_options & c_stereo_calibration::stereo_calibrate_options() const
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

bool c_stereo_calibration::serialize(c_config_setting settings, bool save)
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
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_rectified_frames);
    SERIALIZE_OPTION(section, save, output_options_, rectified_frames_file_name);

    SERIALIZE_OPTION(section, save, output_options_, save_stereo_rectified_frames);
    SERIALIZE_OPTION(section, save, output_options_, stereo_rectified_frames_file_name);

    SERIALIZE_OPTION(section, save, output_options_, save_quad_rectified_frames);
    SERIALIZE_OPTION(section, save, output_options_, quad_rectified_frames_file_name);
  }

  return true;
}


bool c_stereo_calibration::initialize()
{
  calibration_flags_ = calibration_options_.calibration_flags;
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

  return true;
}

void c_stereo_calibration::cleanup()
{
  object_points_.clear();
  current_object_points_.clear();
  rvecs_.clear();
  tvecs_.clear();
  perViewErrors_.release();

  for ( int i = 0; i < 2; ++i ) {

    image_points_[i].clear();
    current_image_points_[i].clear();
    current_frames_[i].release();
    current_masks_[i].release();
    rmaps_[i].release();
  }

  display_frame_.release();
  display_mask_.release();
}

bool c_stereo_calibration::process_stereo_frame(const cv::Mat images[2], cv::Mat masks[2])
{
  bool fOk = true;

  for( int i = 0; i < 2; ++i ) {
    if( images[i].data != current_frames_[i].data ) {
      images[i].copyTo(current_frames_[i]);
    }
    if( masks[i].data != current_masks_[i].data ) {
      masks[i].copyTo(current_masks_[i]);
    }
  }

  for( int i = 0; i < 2; ++i ) {
    current_image_points_[i].clear();
  }

  for( int i = 0; i < 2; ++i ) {
    if( canceled() || !detect_chessboard(current_frames_[i], current_image_points_[i]) ) {
      CF_ERROR("detect_chessboard() fails for source %d", i);
      fOk = false;
      break;
    }
  }

  if( !fOk ) {
    return true; // wait for next frame
  }

  image_points_[0].emplace_back(current_image_points_[0]);
  image_points_[1].emplace_back(current_image_points_[1]);
  object_points_.emplace_back(current_object_points_);

  if( object_points_.size() >= std::max(1, calibration_options_.min_frames) ) {

    const cv::Size image_size =
        current_frames_[0].size();

    fOk = false;

    if( !stereo_intrinsics_initialized_ ) {

      stereo_intrinsics_initialized_ =
          init_camera_intrinsics(stereo_intrinsics_,
              object_points_,
              image_points_[0],
              image_points_[1],
              image_size,
              1);

      if( !stereo_intrinsics_initialized_ ) {
        CF_ERROR("init_camera_intrinsics() fails");
        return true; // wait for next frame
      }
    }

    CF_DEBUG("Running stereo calibration ...");

    rmse_ =
        stereo_calibrate(object_points_,
            image_points_[0], image_points_[1],
            stereo_intrinsics_,
            stereo_extrinsics_,
            calibration_flags_,
            calibration_options_.solverTerm,
            &E_,
            &F_,
            &rvecs_,
            &tvecs_,
            &perViewErrors_);

    CF_DEBUG("done with RMS error=%g", rmse_);

    fOk = rmse_ >= 0;

    const cv::Matx33d &M0 =
        stereo_intrinsics_.camera[0].camera_matrix;

    const cv::Matx33d &M1 =
        stereo_intrinsics_.camera[1].camera_matrix;

    CF_DEBUG("\nM0: {\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "}\n"

        "\nM1: {\n"
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

    if( !fOk || canceled() ) {
      return true; // wait for next frame
    }

    update_state();

    if( canceled() ) {
      return false;
    }

    if( !save_current_camera_parameters() ) {
      CF_ERROR("save_current_camera_parameters() fails");
      return false;
    }

    if( canceled() ) {
      return false;
    }

    update_undistortion_remap();
  }

  return true;
}


bool c_stereo_calibration::get_display_image(cv::Mat *display_frame, cv::Mat * display_mask)
{
  if ( display_frame ) {
    display_frame_.copyTo(*display_frame);
  }
  if ( display_mask ) {
    display_mask_.copyTo(*display_mask);
  }
  return true;
}

bool c_stereo_calibration::canceled()
{
  return false;
}

bool c_stereo_calibration::detect_chessboard(const cv::Mat & frame, std::vector<cv::Point2f> & corners_) const
{
  return find_chessboard_corners(frame,
      chessboard_detection_options_.chessboard_size,
      corners_,
      chessboard_detection_options_);
}

double c_stereo_calibration::estimate_grid_subset_quality(int excludedIndex) const
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

  return mean[0] / (stdDev[0] + 1e-7);
}

void c_stereo_calibration::filter_frames()
{
  const int nbframes =
      object_points_.size();

  const int nbframesmax =
      std::max(1, std::max(calibration_options_.min_frames,
          calibration_options_.max_frames));

  CF_DEBUG("nbframes = %d / %d", nbframes, nbframesmax);

  if( nbframes != perViewErrors_.rows ) {

    if( nbframes > 1 ) {
      CF_ERROR("APP BUG: object_points_.size()=%zu != current_per_view_errors_.total()=%zu",
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

    double worstValue = -HUGE_VAL;
    double maxQuality = estimate_grid_subset_quality(nbframes);

    int worstElemIndex = 0;

    const double alpha =
        calibration_options_.filter_alpha;

    for( size_t i = 0; i < nbframes; i++ ) {

      const double gridQDelta =
          estimate_grid_subset_quality(i) - maxQuality;

      const double currentValue =
          (perViewErrors_[i][0] + perViewErrors_[i][1]) * alpha + gridQDelta * (1. - alpha);

      if( currentValue > worstValue ) {
        worstValue = currentValue;
        worstElemIndex = i;
      }
    }

    CF_DEBUG("XXX worstElemIndex=%d", worstElemIndex);

    image_points_[0].erase(image_points_[0].begin() + worstElemIndex);
    image_points_[1].erase(image_points_[1].begin() + worstElemIndex);
    object_points_.erase(object_points_.begin() + worstElemIndex);

    cv::Mat1f newErrorsVec(nbframes - 1, perViewErrors_.cols);

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

void c_stereo_calibration::update_state()
{
//  cv::Mat &cameraMatrix =
//      current_camera_matrix_;
//
//  if( !cameraMatrix.empty() ) {
//
//    const double relErrEps = 0.05;
//    static const double sigmaMult = 1.96;
//
//    bool fConfState = false;
//    bool cConfState = false;
//    bool dConfState = true;
//
//    const cv::Mat &S =
//        current_std_deviations_;
//
//    if( sigmaMult * S.at<double>(0) / cameraMatrix.at<double>(0, 0) < relErrEps &&
//        sigmaMult * S.at<double>(1) / cameraMatrix.at<double>(1, 1) < relErrEps ) {
//      fConfState = true;
//    }
//
//    if( sigmaMult * S.at<double>(2) / cameraMatrix.at<double>(0, 2) < relErrEps &&
//        sigmaMult * S.at<double>(3) / cameraMatrix.at<double>(1, 2) < relErrEps ) {
//      cConfState = true;
//    }
//
//    for( int i = 0; i < 5; i++ ) {
//      if( S.at<double>(4 + i) / fabs(current_dist_coeffs_.at<double>(i)) > 1 ) {
//        dConfState = false;
//      }
//    }
//
//    confIntervalsState_ =
//        fConfState && cConfState && dConfState;
//  }
//
//
//  if( object_points_.size() > calibration_options_.min_frames ) {
//    coverageQualityState_ = estimate_coverage_quality() > 1.8 ? true : false;
//  }

  if( calibration_options_.auto_tune_calibration_flags && object_points_.size() > calibration_options_.min_frames ) {

//    if( !(calibration_flags_ & cv::CALIB_FIX_ASPECT_RATIO) ) {
//
//      bool fix_aspect_ratio = true;
//
//      for( int i = 0; i < 2; ++i ) {
//
//        const cv::Matx33d & C =
//            stereo_intrinsics_.camera[i].camera_matrix;
//
//        const double fDiff =
//            fabs(C(0, 0) - C(1, 1));
//
//        //        if( fDiff < 3 * S.at<double>(0) && fDiff < 3 * S.at<double>(1) ) {
//        //        }
//      }

//      const cv::Mat &S =
//          current_std_deviations_;
//
//      if( fDiff < 3 * S.at<double>(0) && fDiff < 3 * S.at<double>(1) ) {
//        calibration_flags_ |= cv::CALIB_FIX_ASPECT_RATIO;
//        cameraMatrix.at<double>(0, 0) = cameraMatrix.at<double>(1, 1);
//      }
//    }

    if( !(calibration_flags_ & cv::CALIB_ZERO_TANGENT_DIST) ) {

      const double eps = 0.005;

      bool fix_zero_tangent_dist = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            stereo_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 3) && (fabs(D[2]) > eps || fabs(D[3]) > eps) ) {
          fix_zero_tangent_dist = false;
          break;
        }
      }

      if( fix_zero_tangent_dist ) {
        calibration_flags_ |= cv::CALIB_ZERO_TANGENT_DIST;
      }
    }

    if( !(calibration_flags_ & cv::CALIB_FIX_K1) ) {

      const double eps = 0.005;

      bool fix_k1 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            stereo_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 0) && (fabs(D[0]) > eps) ) {
          fix_k1 = false;
          break;
        }
      }

      if( fix_k1 ) {
        calibration_flags_ |= cv::CALIB_FIX_K1;
      }
    }


    if( !(calibration_flags_ & cv::CALIB_FIX_K2) ) {

      const double eps = 0.005;

      bool fix_k2 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            stereo_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 1) && (fabs(D[1]) > eps) ) {
          fix_k2 = false;
          break;
        }
      }

      if( fix_k2 ) {
        calibration_flags_ |= cv::CALIB_FIX_K2;
      }
    }

    if( !(calibration_flags_ & cv::CALIB_FIX_K3) ) {

      const double eps = 0.005;

      bool fix_k3 = true;

      for( int i = 0; i < 2; ++i ) {

        const std::vector<double> &D =
            stereo_intrinsics_.camera[i].dist_coeffs;

        if( (D.size() > 4) && (fabs(D[4]) > eps) ) {
          fix_k3 = false;
          break;
        }
      }

      if( fix_k3 ) {
        calibration_flags_ |= cv::CALIB_FIX_K3;
      }
    }
  }
}



void c_stereo_calibration::update_undistortion_remap()
{
  const cv::Size & image_size =
      stereo_intrinsics_.camera[0].image_size;

  create_stereo_rectification(image_size,
      stereo_intrinsics_,
      stereo_extrinsics_,
      -1,
      rmaps_,
      &new_stereo_intrinsics_,
      &new_stereo_extrinsics,
      nullptr,
      nullptr,
      nullptr,
      nullptr);

}

void c_stereo_calibration::update_display_image()
{
  const cv::Size sizes[2] = {
      current_frames_[0].size(),
      current_frames_[1].size(),
  };

  const cv::Size size1(sizes[0].width + sizes[1].width,
      std::max(sizes[0].height, sizes[1].height));

  const float side_ratio1 =
      (float) std::min(size1.width, size1.height) /
          (float) std::max(size1.width, size1.height);

  const cv::Size size2(std::max(sizes[0].width, sizes[1].width),
      sizes[0].height + sizes[1].height);

  const float side_ratio2 =
      (float) std::min(size2.width, size2.height) /
          (float) std::max(size2.width, size2.height);

  cv::Rect rc0, rc1;

  if( side_ratio1 * 2 > side_ratio2 ) {

    display_frame_.create(size1, current_frames_[0].type());
    rc0 = cv::Rect(0, 0, sizes[0].width, sizes[0].height);
    rc1 = cv::Rect(sizes[0].width, 0, sizes[1].width, sizes[1].height);

  }
  else {

    display_frame_.create(size2, current_frames_[0].type());
    rc0 = cv::Rect(0, 0, sizes[0].width, sizes[0].height);
    rc1 = cv::Rect(0, sizes[0].height, sizes[1].width, sizes[1].height);
  }

  if( !rmaps_[0].empty() ) {

    cv::remap(current_frames_[0], display_frame_(rc0),
        rmaps_[0], cv::noArray(),
        cv::INTER_LINEAR);

    cv::remap(current_frames_[1], display_frame_(rc1),
        rmaps_[1], cv::noArray(),
        cv::INTER_LINEAR);

  }
  else {

    current_frames_[0].copyTo(display_frame_(rc0));
    current_frames_[1].copyTo(display_frame_(rc1));

    if( display_frame_.channels() == 1 ) {
      if( !current_image_points_[0].empty() || !current_image_points_[1].empty() ) {
        cv::cvtColor(display_frame_, display_frame_,
            cv::COLOR_GRAY2BGR);
      }
    }

    if( !current_image_points_[0].empty() ) {
      cv::drawChessboardCorners(display_frame_(rc0),
          chessboard_detection_options_.chessboard_size,
          current_image_points_[0],
          true);
    }

    if( !current_image_points_[1].empty() ) {
      cv::drawChessboardCorners(display_frame_(rc1),
          chessboard_detection_options_.chessboard_size,
          current_image_points_[1],
          true);
    }
  }
}

bool c_stereo_calibration::save_current_camera_parameters() const
{
  if( !output_intrinsics_filename_.empty() ) {

    if( !create_path(get_parent_directory(output_intrinsics_filename_)) ) {
      CF_ERROR("create_path('%s') fails: %s", output_intrinsics_filename_.c_str(), strerror(errno));
      return false;
    }

    if( !write_stereo_camera_intrinsics_yml(stereo_intrinsics_, output_intrinsics_filename_) ) {
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

    if( !write_stereo_camera_extrinsics_yml(stereo_extrinsics_, output_extrinsics_filename_) ) {
      CF_ERROR("ERROR: save_stereo_camera_extrinsics_yml('%s') fails",
          output_extrinsics_filename_.c_str());
    }
  }

  return true;
}
