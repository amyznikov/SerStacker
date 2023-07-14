/*
 * QStereoCalibrationPipeline.cc
 *
 *  Created on: Jul 6, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrationPipeline.h"

QStereoCalibrationSettingsWidget::QStereoCalibrationSettingsWidget(QWidget * parent) :
  ThisClass("", parent)
{
}

QStereoCalibrationSettingsWidget::QStereoCalibrationSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  ///

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QStereoCalibrationInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Chessboard Detection Options",
      chessboardCornersDetection_ctl = new QChessboardCornersDetectionOptions(this));
  connect(chessboardCornersDetection_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Calibrate Options",
      stereoCalibrateOptions_ctl = new QStereoCalibrateOptions(this));
  connect(stereoCalibrateOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Output Options",
      outputOptions_ctl = new QStereoCalibrationOutputOptions(this));
  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  updateControls();
}

void QStereoCalibrationSettingsWidget::update_pipeline_controls()
{
  Base::update_pipeline_controls();

  if ( !pipeline_ ) {
    inputOptions_ctl->set_pipeline(nullptr);
    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(nullptr);
    stereoCalibrateOptions_ctl->set_calibration_options(nullptr);
    outputOptions_ctl->set_output_options(nullptr);
  }
  else {
    inputOptions_ctl->set_pipeline(pipeline_);
    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(&pipeline_->chessboard_detection_options());
    stereoCalibrateOptions_ctl->set_calibration_options(&pipeline_->calibration_options());
    outputOptions_ctl->set_output_options(&pipeline_->output_options());
  }
}
