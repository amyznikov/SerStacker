/*
 * QLiveStereoCalibrationPipeline.cc
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#include "QLiveStereoCalibrationPipeline.h"

namespace serimager {

QLiveStereoCalibrationPipeline::QLiveStereoCalibrationPipeline(const QString & name, QObject * parent) :
    Base(name, parent)
{
}

bool QLiveStereoCalibrationPipeline::serialize(c_config_setting settings, bool save)
{
  if ( !Base::serialize(settings, save) ) {
    CF_ERROR("QLiveStereoCalibrationPipeline::Base::serialize() fails");
    return false;
  }

  if ( !c_stereo_calibration::serialize(settings, save) ) {
    CF_ERROR("c_stereo_calibration::serialize() fails");
    return false;
  }

  return true;
}

bool QLiveStereoCalibrationPipeline::initialize_pipeline()
{
  if ( !Base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  output_path_ =
      create_output_path(output_options().output_directory);

  /////////////////////////////////////////////////////////////////////////////

  const std::string dateTimeString =
      QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString();

  c_stereo_calibration::set_output_intrinsics_filename(
      ssprintf("%s/stereo_intrinsics.%s.%s.yml",
          output_path_.c_str(),
          name_.toUtf8().constData(),
          dateTimeString.c_str()));

  c_stereo_calibration::set_output_extrinsics_filename(
      ssprintf("%s/stereo_extrinsics.%s.%s.yml",
          output_path_.c_str(),
          name_.toUtf8().constData(),
          dateTimeString.c_str()));


  if( output_options().save_chessboard_frames ) {
    c_stereo_calibration::set_chessboard_frames_filename(
        generate_output_file_name(output_path_,
            output_options().chessboard_frames_filename,
            "live",
            ".avi"));
  }

  if ( !c_stereo_calibration::initialize() ) {
    CF_ERROR("c_stereo_calibration::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}

void QLiveStereoCalibrationPipeline::cleanup_pipeline()
{
  c_stereo_calibration::cleanup();
  Base::cleanup_pipeline();
}


bool QLiveStereoCalibrationPipeline::process_frame(const cv::Mat & image, COLORID colorid, int bpp)
{
  cv::Mat currentImage;
  cv::Rect roi[2];

  displayColorid_ =
      colorid == COLORID_MONO ? COLORID_MONO :
          COLORID_BGR;

  if( !Base::convert_image(image, colorid, bpp, &currentImage, displayColorid_, CV_8U) ) {
    CF_ERROR("convertInputImage() fails");
    return false;
  }


  roi[0] = cv::Rect(0, 0, currentImage.cols / 2, currentImage.rows);
  roi[1] = cv::Rect(currentImage.cols / 2, 0, currentImage.cols / 2, currentImage.rows);
  current_frames_[0] = currentImage(roi[0]);
  current_frames_[1] = currentImage(roi[1]);

  if( !process_current_stereo_frame(calibration_options().enable_calibration) ) {
    CF_ERROR("process_current_stereo_frame() fails");
    return false;
  }

  return true;
}

bool QLiveStereoCalibrationPipeline::get_display_image(cv::Mat * displayImage, COLORID * colorid, int * bpp)
{
  *colorid = displayColorid_;
  *bpp = 8;

  if( !c_stereo_calibration::get_display_image(*displayImage, cv::noArray()) ) {
    CF_ERROR("stereo_calibration_.get_display_image() fails");
    return false;
  }

  return true;
}

} /* namespace serimager */
