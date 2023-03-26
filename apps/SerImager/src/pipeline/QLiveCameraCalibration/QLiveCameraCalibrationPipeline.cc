/*
 * QLiveCameraCalibrationPipeline.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "QLiveCameraCalibrationPipeline.h"

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
      create_output_path(camera_calibration_.output_options().output_directory.c_str());

  /////////////////////////////////////////////////////////////////////////////

  camera_calibration_.set_output_intrinsics_filename(
      ssprintf("%s/camera_intrinsics.%s.yml",
          output_path_.toUtf8().constData(),
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
