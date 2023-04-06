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
  add_expandable_groupbox("Chessboard Corners Detection",
      chessboardDetectionOptions_ctl = new QChessboardCornersDetectionOptions(this));
  connect(chessboardDetectionOptions_ctl, &QSettingsWidget::parameterChanged,
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


void QCameraCalibrationOptions::set_camera_calibration(c_camera_calibration * options)
{
  options_ = options;
  updateControls();
}

c_camera_calibration * QCameraCalibrationOptions::camera_calibration() const
{
  return options_;
}

QChessboardCornersDetectionOptions * QCameraCalibrationOptions::chessboardDetectionOptions() const
{
  return chessboardDetectionOptions_ctl;
}

QCalibrateCameraOptions * QCameraCalibrationOptions::calibrateCameraOptions() const
{
  return calibrateCameraOptions_ctl;
}

QCameraCalibrationOutputOptions * QCameraCalibrationOptions::outputOptions() const\
{
  return outputOptions_ctl;
}

void QCameraCalibrationOptions::onupdatecontrols()
{
  if ( !options_ ) {

    setEnabled(false);

    chessboardDetectionOptions_ctl->set_chessboard_corners_detection_options(nullptr);
    calibrateCameraOptions_ctl->set_options(nullptr);
    outputOptions_ctl->set_options(nullptr);
  }
  else {

    chessboardDetectionOptions_ctl->set_chessboard_corners_detection_options(
        &options_->chessboard_corners_detection_options());

    calibrateCameraOptions_ctl->set_options(&options_->calibrate_camera_options());
    outputOptions_ctl->set_options(&options_->output_options());

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
