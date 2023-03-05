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


c_stereo_calibration_pipeline::c_stereo_calibration_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

c_stereo_calibration_pipeline::~c_stereo_calibration_pipeline()
{
  cancel();
}


void c_stereo_calibration_pipeline::set_chessboard_size(const cv::Size & v)
{
  chessboard_size_ = v;
}

const cv::Size& c_stereo_calibration_pipeline::chessboard_size() const
{
  return chessboard_size_;
}

void c_stereo_calibration_pipeline::set_chessboard_cell_size(const cv::Size2f & v)
{
  chessboard_cell_size_ = v;
}

const cv::Size2f & c_stereo_calibration_pipeline::chessboard_cell_size() const
{
  return chessboard_cell_size_;
}

c_stereo_calibration_input_options & c_stereo_calibration_pipeline::input_options()
{
  return input_options_;
}

const c_stereo_calibration_input_options & c_stereo_calibration_pipeline::input_options() const
{
  return input_options_;
}

c_chessboard_corners_detection_options & c_stereo_calibration_pipeline::chessboard_corners_detection_options()
{
  return chessboard_corners_detection_options_;
}

const c_chessboard_corners_detection_options & c_stereo_calibration_pipeline::chessboard_corners_detection_options() const
{
  return chessboard_corners_detection_options_;
}

c_stereo_calibrate_options & c_stereo_calibration_pipeline::stereo_calibrate_options()
{
  return calibration_options_;
}

const c_stereo_calibrate_options & c_stereo_calibration_pipeline::stereo_calibrate_options() const
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
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  SERIALIZE_PROPERTY(settings, save, *this, chessboard_size);
  SERIALIZE_PROPERTY(settings, save, *this, chessboard_cell_size);

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, input_options_, left_stereo_source);
    SERIALIZE_OPTION(section, save, input_options_, right_stereo_source);
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

bool c_stereo_calibration_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  lock_guard lock(display_lock_);
  display_frame_.copyTo(frame);
  mask.release();
  return true;
}

void c_stereo_calibration_pipeline::update_output_path()
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


bool c_stereo_calibration_pipeline::read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const
{
  lock_guard lock(display_lock_);

  INSTRUMENT_REGION("");

  enum COLORID colorid = COLORID_UNKNOWN;
  int bpp = 0;


  if ( !source->read(output_image, &colorid, &bpp) ) {
    CF_FATAL("source->read() fails\n");
    return false;
  }

  if ( is_bayer_pattern(colorid) ) {

    DEBAYER_ALGORITHM algo =
        default_debayer_algorithm();

    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, colorid, algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, colorid) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }

        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << bpp)));

        if ( !nninterpolation(output_image, output_image, colorid) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toString(algo));
        return false;
    }
  }
  else if ( colorid == COLORID_OPTFLOW || (output_image.channels() != 4 && output_image.channels() != 2) ) {
    output_mask.release();
  }
  else if( !splitbgra(output_image, output_image, &output_mask) ) {
    output_mask.release();
    return false;
  }

  if( input_options_.enable_color_maxtrix && source->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        source->color_matrix());
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


bool c_stereo_calibration_pipeline::detect_chessboard(const cv::Mat &frame, std::vector<cv::Point2f> & corners_)
{
  bool is_chessboard_found =
      find_chessboard_corners(frame,
          chessboard_size_,
          corners_,
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

//  if( !is_chessboard_found_ ) {
//    CF_ERROR("find_chessboard_corners() fails");
//    return false;
//  }

  return is_chessboard_found;
}


void c_stereo_calibration_pipeline::update_undistortion_remap()
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


double c_stereo_calibration_pipeline::estimate_grid_subset_quality(int excludedIndex) const
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

void c_stereo_calibration_pipeline::filter_frames()
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

void c_stereo_calibration_pipeline::update_state()
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


bool c_stereo_calibration_pipeline::save_current_camera_parameters() const
{
  //
  // Save computed intrinsics and extrinsics into yml
  //

  if( !create_path(output_path_) ) {
    CF_ERROR("create_path('%s') fails: %s ", output_path_.c_str(), strerror(errno));
    return false;
  }

  const std::string intrinsics_file_name =
      ssprintf("%s/stereo_intrinsics.%s.yml",
          output_path_.c_str(),
          csequence_name());

  if( !write_stereo_camera_intrinsics_yml(stereo_intrinsics_, intrinsics_file_name) ) {
    CF_ERROR("ERROR: save_stereo_camera_intrinsics_yml('%s') fails",
        intrinsics_file_name.c_str());
  }

  const std::string extrinsics_file_name =
      ssprintf("%s/stereo_extrinsics.%s.yml",
          output_path_.c_str(),
          csequence_name());

  if( !write_stereo_camera_extrinsics_yml(stereo_extrinsics_, extrinsics_file_name) ) {
    CF_ERROR("ERROR: save_stereo_camera_extrinsics_yml('%s') fails",
        extrinsics_file_name.c_str());
  }

  return true;
}


void c_stereo_calibration_pipeline::update_display_image()
{
  if ( true ) {
    lock_guard lock(display_lock_);

    accumulated_frames_ =
        object_points_.size();

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
            chessboard_size_,
            current_image_points_[0],
            true);
      }

      if( !current_image_points_[1].empty() ) {
        cv::drawChessboardCorners(display_frame_(rc1),
            chessboard_size_,
            current_image_points_[1],
            true);
      }

    }
  }

  on_accumulator_changed();
}

bool c_stereo_calibration_pipeline::initialize_pipeline()
{
  set_pipeline_stage(stereo_calibration_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  calibration_flags_ = calibration_options_.calibration_flags;
  stereo_intrinsics_initialized_ = false;

  /////////////////////////////////////////////////////////////////////////////

  if ( chessboard_size_.width < 2 || chessboard_size_.height < 2 ) {
    CF_ERROR("Invalid chessboard_size_: %dx%d", chessboard_size_.width, chessboard_size_.height);
    return false;
  }

  if ( !(chessboard_cell_size_.width > 0) || !(chessboard_cell_size_.height > 0)  ) {
    CF_ERROR("Invalid chessboard_cell_size_: %gx%g", chessboard_cell_size_.width, chessboard_cell_size_.height);
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  if ( input_options_.left_stereo_source.empty() ) {
    CF_ERROR("ERROR: No left stereo source specified");
    return false;
  }

  if ( input_options_.right_stereo_source.empty() ) {
    CF_ERROR("ERROR: No right stereo source specified");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  input_sources_[0] =
      input_sequence_->source(input_options_.left_stereo_source);

  if ( !input_sources_[0] ) {
    CF_ERROR("ERROR: requested left stereo source not found in input sequence: %s",
        input_options_.left_stereo_source.c_str());
    return false;
  }

  input_sources_[1] =
      input_sequence_->source(input_options_.right_stereo_source);

  if ( !input_sources_[1] ) {
    CF_ERROR("ERROR: requested right stereo source not found in input sequence: %s",
        input_options_.right_stereo_source.c_str());
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
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

void c_stereo_calibration_pipeline::cleanup_pipeline()
{
  set_pipeline_stage(stereo_calibration_finishing);

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

    if ( input_sources_[i] ) {
      input_sources_[i]->close();
      input_sources_[i].reset();
    }
  }


  if( true ) {
    lock_guard lock(display_lock_);
    display_frame_.release();
    display_mask_.release();
  }

  base::cleanup_pipeline();

  set_pipeline_stage(stereo_calibration_idle);
}

bool c_stereo_calibration_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  for ( int i = 0; i < 2; ++i ) {
    if ( !input_sources_[i]->open() ) {
      CF_ERROR("ERROR: can not open input source '%s'", input_sources_[i]->cfilename());
      return false;
    }
  }

  if( input_sources_[0]->size() != input_sources_[1]->size() ) {
    CF_ERROR("ERROR: input sources sizes not match: left size=%d right size=%d ",
        input_sources_[0]->size(), input_sources_[1]->size());
    return false;
  }

  const int start_pos =
      std::max(input_options_.start_frame_index, 0);

  const int end_pos =
      input_options_.max_input_frames < 1 ?
          input_sources_[0]->size() :
          std::min(input_sources_[0]->size(),
              input_options_.start_frame_index + input_options_.max_input_frames);


  total_frames_ = end_pos - start_pos;
  processed_frames_ = 0;
  accumulated_frames_ = 0;

  if( total_frames_ < 1 ) {
    CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
        total_frames_);
    return false;
  }


  for( int i = 0; i < 2; ++i ) {
    if( !input_sources_[i]->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sources_[%d]->seek(start_pos=%d) fails", i, start_pos);
      return false;
    }
  }

  set_pipeline_stage(stereo_calibration_in_progress);
  set_status_msg("RUNNING ...");

  bool fOk;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

    if ( canceled() ) {
      break;
    }

    filter_frames();

    if ( canceled() ) {
      break;
    }

    fOk = true;
    for( int i = 0; i < 2; ++i ) {

      if( !read_input_frame(input_sources_[i], current_frames_[i], current_masks_[i]) ) {
        CF_ERROR("ERROR: read_input_frame() fails for source %d", i);
        fOk = false;
        break;
      }
    }

    if ( !fOk || canceled() ) {
      break;
    }

    fOk = true;

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

    if ( !fOk ) {
      continue;
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

        if ( !stereo_intrinsics_initialized_ ) {
          CF_ERROR("init_camera_intrinsics() fails");
          continue;
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

      CF_DEBUG("done with RMS error=%g",  rmse_);

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
        continue;
      }

      update_state();

      if( canceled() ) {
        break;
      }

      if( !save_current_camera_parameters() ) {
        CF_ERROR("save_current_camera_parameters() fails");
        return false;
      }

      if( canceled() ) {
        break;
      }

      update_undistortion_remap();
    }

    if( canceled() ) {
      break;
    }

    update_display_image();
  }

  for ( int i = 0; i < 2; ++i ) {
    input_sources_[i]->close();
  }

  if( !canceled() && output_options_.save_rectified_images ) {

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


    for( int i = 0; i < 2; ++i ) {
      if( !input_sources_[i]->open() ) {
        CF_ERROR("ERROR: can not open input source '%s'", input_sources_[i]->cfilename());
        return false;
      }
    }

    CF_DEBUG("Saving %s...", output_file_name.c_str());

    total_frames_ = input_sources_[0]->size();
    processed_frames_ = 0;
    accumulated_frames_ = 0;

    for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

      if( canceled() ) {
        break;
      }

      fOk = true;

      for( int i = 0; i < 2; ++i ) {

        if( !read_input_frame(input_sources_[i], current_frames_[i], current_masks_[i]) ) {
          CF_ERROR("ERROR: read_input_frame() fails for source %d", i);
          fOk = false;
          break;
        }

        if( canceled() ) {
          break;
        }
      }

      if( !fOk || canceled() ) {
        break;
      }

      update_display_image();

      if( canceled() ) {
        break;
      }

      if( !writer.is_open() ) {

        fOk =
            writer.open(output_file_name,
                display_frame_.size(),
                display_frame_.channels() > 1,
                false);

        if( !fOk ) {
          CF_ERROR("writer.open('%s') fails", output_file_name.c_str());
          return false;
        }
      }

      if( !writer.write(display_frame_, cv::noArray(), false, processed_frames_) ) {
        CF_ERROR("c_video_writer: write(fails)");
        break;
      }

      if( canceled() ) {
        break;
      }
    }
  }

  return true;
}



