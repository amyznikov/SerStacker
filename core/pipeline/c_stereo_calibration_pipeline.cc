/*
 * c_stereo_calibration_pipeline.cc
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#include "c_stereo_calibration_pipeline.h"
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

template<>
const c_enum_member* members_of<STEREO_CALIBRATION_FLAGS>()
{
  static constexpr c_enum_member members[] = {
      {STEREO_CALIB_FIX_INTRINSIC, "FIX_INTRINSIC","Fix cameraMatrix? and distCoeffs? so that only R, T, E, and F matrices are estimated."},
      {STEREO_CALIB_USE_INTRINSIC_GUESS, "USE_INTRINSIC_GUESS"," Optimize some or all of the intrinsic parameters according to the specified flags. Initial values are provided by the user."},
      {STEREO_CALIB_USE_EXTRINSIC_GUESS, "USE_EXTRINSIC_GUESS"," R and T contain valid initial values that are optimized further. Otherwise R and T are initialized to the median value of the pattern views (each dimension separately)."},
      {STEREO_CALIB_FIX_PRINCIPAL_POINT, "FIX_PRINCIPAL_POINT"," Fix the principal points during the optimization."},
      {STEREO_CALIB_FIX_FOCAL_LENGTH, "FIX_FOCAL_LENGTH"," Fix f(j)x and f(j)y ."},
      {STEREO_CALIB_FIX_ASPECT_RATIO, "FIX_ASPECT_RATIO"," Optimize f(j)y . Fix the ratio f(j)x/f(j)y"},
      {STEREO_CALIB_SAME_FOCAL_LENGTH, "SAME_FOCAL_LENGTH"," Enforce f(0)x=f(1)x and f(0)y=f(1)y ."},
      {STEREO_CALIB_ZERO_TANGENT_DIST, "ZERO_TANGENT_DIST"," Set tangential distortion coefficients for each camera to zeros and fix there."},
      {STEREO_CALIB_FIX_K1, "FIX_K1","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K2, "FIX_K2","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K3, "FIX_K3","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K4, "FIX_K4","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K5, "FIX_K5","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_FIX_K6, "FIX_K6","Do not change the corresponding radial distortion coefficient during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_RATIONAL_MODEL, "RATIONAL_MODEL"," Enable coefficients k4, k5, and k6. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the rational model and return 8 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients."},
      {STEREO_CALIB_THIN_PRISM_MODEL, "THIN_PRISM_MODEL"," Coefficients s1, s2, s3 and s4 are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the thin prism model and return 12 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients."},
      {STEREO_CALIB_FIX_S1_S2_S3_S4, "FIX_S1_S2_S3_S4"," The thin prism distortion coefficients are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      {STEREO_CALIB_TILTED_MODEL, "TILTED_MODEL"," Coefficients tauX and tauY are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the tilted sensor model and return 14 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients."},
      {STEREO_CALIB_FIX_TAUX_TAUY, "FIX_TAUX_TAUY"," The coefficients of the tilted sensor model are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0."},
      { 0 }
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
  return false;
}

void c_stereo_calibration_pipeline::update_output_path()
{
  if( output_directory_.empty() ) {

    std::string parent_directory =
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
        "./stereo-calib";
  }
}


void c_stereo_calibration_pipeline::set_pipeline_stage(STEREO_CALIBRATION_STAGE stage)
{
  const auto oldstage = pipeline_stage_;

  if ( stage != oldstage ) {
    pipeline_stage_ = stage;
    on_pipeline_stage_changed(oldstage, stage);
  }
}


bool c_stereo_calibration_pipeline::read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const
{
  return false;
}


bool c_stereo_calibration_pipeline::detect_chessboard(const cv::Mat &frame)
{
  return false;
}


void c_stereo_calibration_pipeline::update_undistortion_remap()
{
}


void c_stereo_calibration_pipeline::update_display_image()
{
}

bool c_stereo_calibration_pipeline::initialize_pipeline()
{
  set_pipeline_stage(stereo_calibration_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_chessboard_camera_calibration_pipeline: base::initialize() fails");
    return false;
  }

  is_chessboard_found_ = false;


  calibration_flags_ = calibration_options_.calibration_flags;

  if ( chessboard_size_.width < 2 || chessboard_size_.height < 2 ) {
    CF_ERROR("Invalid chessboard_size_: %dx%d", chessboard_size_.width, chessboard_size_.height);
    set_status_msg("ERROR: Invalid chessboard_size specified");
    return false;
  }

  return true;
}

void c_stereo_calibration_pipeline::cleanup_pipeline()
{
  set_pipeline_stage(stereo_calibration_finishing);

  base::cleanup_pipeline();


  set_pipeline_stage(stereo_calibration_idle);
}

bool c_stereo_calibration_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s' ...",
      cname());

  return false;
}



