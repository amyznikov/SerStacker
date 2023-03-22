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

c_stereo_calibration & QLiveStereoCalibrationPipeline::stereo_calibration()
{
  return stereo_calibration_;
}

const c_stereo_calibration & QLiveStereoCalibrationPipeline::stereo_calibration() const
{
  return stereo_calibration_;
}

bool QLiveStereoCalibrationPipeline::serialize(c_config_setting settings, bool save)
{
  if ( !Base::serialize(settings, save) ) {
    CF_ERROR("QLiveStereoCalibrationPipeline::Base::serialize() fails");
    return false;
  }

  if ( !stereo_calibration_.serialize(settings, save) ) {
    CF_ERROR("stereo_calibration_.serialize() fails");
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
      create_output_path(stereo_calibration_.output_options().output_directory.c_str());

  /////////////////////////////////////////////////////////////////////////////

  stereo_calibration_.set_output_intrinsics_filename(
      ssprintf("%s/stereo_intrinsics.%s.yml",
          output_path_.toUtf8().constData(),
          name_.toUtf8().constData()));

  stereo_calibration_.set_output_extrinsics_filename(
      ssprintf("%s/stereo_extrinsics.%s.yml",
          output_path_.toUtf8().constData(),
          name_.toUtf8().constData()));

  if ( !stereo_calibration_.initialize() ) {
    CF_ERROR("stereo_calibration_.initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}

void QLiveStereoCalibrationPipeline::cleanup_pipeline()
{
  stereo_calibration_.cleanup();
  Base::cleanup_pipeline();
}


bool QLiveStereoCalibrationPipeline::process_frame(const cv::Mat & image, COLORID colorid, int bpp)
{
  cv::Mat currentImage;
  cv::Mat frames[2];
  cv::Mat masks[2];
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
  frames[0] = currentImage(roi[0]);
  frames[1] = currentImage(roi[1]);

  if ( !stereo_calibration_.process_stereo_frame(frames, masks) ) {
    CF_ERROR("stereo_calibration_.process_stereo_frame() fails");
    return false;
  }

  //  displayImage_ =
  //      currentImage;

  return true;
}

bool QLiveStereoCalibrationPipeline::get_display_image(cv::Mat * displayImage, COLORID * colorid, int * bpp)
{
  *colorid = displayColorid_;
  *bpp = 8;

  stereo_calibration_.update_display_image();

  if( !stereo_calibration_.get_display_image(displayImage, nullptr) ) {
    CF_ERROR("stereo_calibration_.get_display_image() fails");
    return false;
  }

  return true;
}

} /* namespace serimager */
