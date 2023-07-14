/*
 * QCameraCalibrationPipeline.cc
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#include "QCameraCalibrationPipeline.h"


QCameraCalibrationSettingsWidget::QCameraCalibrationSettingsWidget(QWidget * parent) :
  ThisClass("", parent)
{
}

QCameraCalibrationSettingsWidget::QCameraCalibrationSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  ///

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QCameraCalibrationInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

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

  ///

  updateControls();
}

void QCameraCalibrationSettingsWidget::update_pipeline_controls()
{
  Base::update_pipeline_controls();

  if ( !pipeline_ ) {
    inputOptions_ctl->set_pipeline(nullptr);
    chessboardDetectionOptions_ctl->set_chessboard_corners_detection_options(nullptr);
    calibrateCameraOptions_ctl->set_options(nullptr);
    outputOptions_ctl->set_output_options(nullptr);
  }
  else {
    inputOptions_ctl->set_pipeline(pipeline_);
    chessboardDetectionOptions_ctl->set_chessboard_corners_detection_options(&pipeline_->chessboard_corners_detection_options());
    calibrateCameraOptions_ctl->set_options(&pipeline_->calibrate_camera_options());
    outputOptions_ctl->set_output_options(&pipeline_->output_options());
  }
}
