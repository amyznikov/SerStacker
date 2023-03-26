/*
 * QLiveCameraCalibrationPipeline.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "QLiveCameraCalibrationPipeline.h"
#include <core/settings/opencv_settings.h>

namespace serimager {

QLiveCameraCalibrationPipeline::QLiveCameraCalibrationPipeline(const QString & name, QObject * parent) :
    Base(name, parent)
{
}

c_camera_calibration & QLiveCameraCalibrationPipeline::camera_calibration()
{
  return camera_calibration_;
}

const c_camera_calibration & QLiveCameraCalibrationPipeline::camera_calibration() const
{
  return camera_calibration_;
}

void QLiveCameraCalibrationPipeline::set_save_frames_with_detected_chessboard(bool v)
{
  save_frames_with_detected_chessboard_ = v;
}

bool QLiveCameraCalibrationPipeline::save_frames_with_detected_chessboard() const
{
  return save_frames_with_detected_chessboard_;
}

void QLiveCameraCalibrationPipeline::set_frames_with_detected_chessboard_filename(const QString & v)
{
  frames_with_detected_chessboard_filename_ = v;
}

const QString & QLiveCameraCalibrationPipeline::frames_with_detected_chessboard_filename() const
{
  return frames_with_detected_chessboard_filename_;
}

bool QLiveCameraCalibrationPipeline::serialize(c_config_setting settings, bool save)
{
  if ( !Base::serialize(settings, save) ) {
    CF_ERROR("QLiveCameraCalibrationPipeline::Base::serialize() fails");
    return false;
  }

  if ( !camera_calibration_.serialize(settings, save) ) {
    CF_ERROR("camera_calibration_.serialize() fails");
    return false;
  }

  if( save ) {
    ::save_settings(settings, "save_frames_with_detected_chessboard",
        save_frames_with_detected_chessboard_);
    ::save_settings(settings, "frames_with_detected_chessboard_filename",
        frames_with_detected_chessboard_filename_.toStdString());
  }
  else {
    std::string s;

    ::load_settings(settings, "save_frames_with_detected_chessboard",
        &save_frames_with_detected_chessboard_);

    if( ::load_settings(settings, "frames_with_detected_chessboard_filename", &s) && !s.empty() ) {
      frames_with_detected_chessboard_filename_ = s.c_str();
    }
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
      create_output_path(camera_calibration_.output_options().output_directory);

  /////////////////////////////////////////////////////////////////////////////

  camera_calibration_.set_output_intrinsics_filename(
      ssprintf("%s/camera_intrinsics.%s.yml",
          output_path_.c_str(),
          name_.toUtf8().constData()));

  if ( !camera_calibration_.initialize() ) {
    CF_ERROR("camera_calibration_.initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}

void QLiveCameraCalibrationPipeline::cleanup_pipeline()
{
  frame_writer_.close();
  camera_calibration_.cleanup();
  Base::cleanup_pipeline();
}


bool QLiveCameraCalibrationPipeline::process_frame(const cv::Mat & image, COLORID colorid, int bpp)
{
  cv::Mat currentImage;
  cv::Mat frame;
  cv::Mat mask;

  displayColorid_ =
      colorid == COLORID_MONO ? COLORID_MONO :
          COLORID_BGR;

  if( !Base::convert_image(image, colorid, bpp, &currentImage, displayColorid_, CV_8U) ) {
    CF_ERROR("convertInputImage() fails");
    return false;
  }

  frame = currentImage;

  if ( !camera_calibration_.process_frame(frame, mask) ) {
    CF_ERROR("camera_calibration_.process_frame() fails");
    return false;
  }

  if ( save_frames_with_detected_chessboard_ && camera_calibration_.is_chessboard_found() ) {

    if( !frame_writer_.is_open() ) {

      std::string output_file_name =
          generate_output_file_name(output_path_,
              frames_with_detected_chessboard_filename_.toStdString(),
              "live",
              ".avi");

      bool fOK =
          frame_writer_.open(output_file_name,
              currentImage.size(),
              currentImage.channels() > 1,
              false);

      if( !fOK ) {
        CF_ERROR("display_frame_.open('%s') fails",
            output_file_name.c_str());
        return false;
      }
    }

    if( !frame_writer_.write(currentImage, cv::noArray(), false, 0) ) {
      CF_ERROR("progress_writer.write() fails");
      return false;
    }
  }

  return true;
}

bool QLiveCameraCalibrationPipeline::get_display_image(cv::Mat * displayImage, COLORID * colorid, int * bpp)
{
  *colorid = displayColorid_;
  *bpp = 8;

  //camera_calibration_.update_display_image();

  if( !camera_calibration_.get_display_image(*displayImage, cv::noArray()) ) {
    CF_ERROR("camera_calibration_.get_display_image() fails");
    return false;
  }

  return true;
}

} /* namespace serimager */
