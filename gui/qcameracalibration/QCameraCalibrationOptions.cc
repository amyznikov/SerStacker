/*
 * QCameraCalibrationOptions.cc
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#include "QCameraCalibrationOptions.h"

QCameraCalibrationOptions::QCameraCalibrationOptions(QWidget * parent) :
  Base("QCameraCalibrationSettings", parent)
{
//  add_expandable_groupbox("Input options",
//      inputOptions_ctl = new QCameraCalibrationInputOptions(this));
//  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Chessboard Corners Detection",
      chessboardCornersDetection_ctl = new QChessboardCornersDetectionOptions(this));
  connect(chessboardCornersDetection_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Calibrate Options",
      calibrateCameraOptions_ctl = new QCalibrateCameraOptions(this));
  connect(calibrateCameraOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Output Options",
      outputOptions_ctl = new QCameraCalibrationOutputOptions(this));
  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}


void QCameraCalibrationOptions::set_options(c_camera_calibration * options)
{
  options_ = options;
  updateControls();
}

c_camera_calibration * QCameraCalibrationOptions::options() const
{
  return options_;
}

void QCameraCalibrationOptions::onupdatecontrols()
{
  if ( !options_ ) {

    setEnabled(false);

    //inputOptions_ctl->set_current_pipeline(nullptr);
    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(nullptr);
    calibrateCameraOptions_ctl->set_options(nullptr);
    outputOptions_ctl->set_options(nullptr);
  }
  else {

    //inputOptions_ctl->set_current_pipeline(options_);

    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(
        &options_->chessboard_corners_detection_options());

    calibrateCameraOptions_ctl->set_options(&options_->calibrate_camera_options());
    outputOptions_ctl->set_options(&options_->output_options());

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
