/*
 * QLiveCameraCalibrationPipeline.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "QLiveCameraCalibrationPipeline.h"
#if 0

#include <core/settings/opencv_settings.h>

namespace serimager {

QLiveCameraCalibrationPipeline::QLiveCameraCalibrationPipeline(const QString & name, QObject * parent) :
    Base(name, parent)
{
}

bool QLiveCameraCalibrationPipeline::serialize(c_config_setting settings, bool save)
{
  if ( !Base::serialize(settings, save) ) {
    CF_ERROR("QLiveCameraCalibrationPipeline::Base::serialize() fails");
    return false;
  }

  if( !c_camera_calibration::serialize(settings, save) ) {
    CF_ERROR("c_camera_calibration::serialize() fails");
    return false;
  }

  return true;
}

bool QLiveCameraCalibrationPipeline::initialize_pipeline()
{
  if ( !Base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  output_path_ =
      create_output_path(output_options().output_directory);

  const std::string dateTimeString =
      QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString();


  c_camera_calibration::set_output_intrinsics_filename(
      ssprintf("%s/camera_intrinsics.%s.%s.yml",
          output_path_.c_str(),
          name_.toUtf8().constData(),
          dateTimeString.c_str()));

  if( output_options().save_chessboard_frames ) {
    c_camera_calibration::set_chessboard_frames_filename(
        generate_output_file_name(output_path_,
            output_options().chessboard_frames_filename,
            "live",
            ".avi"));
  }

  if ( !c_camera_calibration::initialize() ) {
    CF_ERROR("c_camera_calibration::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}

void QLiveCameraCalibrationPipeline::cleanup_pipeline()
{
  c_camera_calibration::cleanup();
  Base::cleanup_pipeline();
}


bool QLiveCameraCalibrationPipeline::process_frame(const cv::Mat & image, COLORID colorid, int bpp)
{
  cv::Mat currentImage;

  displayColorid_ =
      colorid == COLORID_MONO ? COLORID_MONO :
          COLORID_BGR;

  if( !Base::convert_image(image, colorid, bpp, &currentImage, displayColorid_, CV_8U) ) {
    CF_ERROR("convertInputImage() fails");
    return false;
  }

  c_camera_calibration::current_frame_ =
      currentImage;

  if( !process_current_frame(calibrate_camera_options().enable_calibration) ) {
    CF_ERROR("process_current_frame() fails");
    return false;
  }

  return true;
}

bool QLiveCameraCalibrationPipeline::get_display_image(cv::Mat * displayImage, COLORID * colorid, int * bpp)
{
  *colorid = displayColorid_;
  *bpp = 8;

   if( !c_camera_calibration::get_display_image(*displayImage, cv::noArray()) ) {
    CF_ERROR("c_camera_calibration::get_display_image() fails");
    return false;
  }

  return true;
}

} /* namespace serimager */

#endif // 0
