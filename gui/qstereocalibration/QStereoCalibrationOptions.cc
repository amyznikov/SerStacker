/*
 * QStereoCalibrationOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrationOptions.h"

QStereoCalibrationOptions::QStereoCalibrationOptions(QWidget * parent) :
  Base("QCameraCalibrationSettings", parent)
{
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


  updateControls();
}


void QStereoCalibrationOptions::set_options(c_stereo_calibration * options)
{
  options_ = options;
  updateControls();
}

c_stereo_calibration * QStereoCalibrationOptions::options() const
{
  return options_;
}

void QStereoCalibrationOptions::onupdatecontrols()
{
  if ( !options_ ) {

    setEnabled(false);

    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(nullptr);
    stereoCalibrateOptions_ctl->set_options(nullptr);
    outputOptions_ctl->set_options(nullptr);
  }
  else {

    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(
        &options_->chessboard_detection_options());

    stereoCalibrateOptions_ctl->set_options(&options_->calibration_options());
    outputOptions_ctl->set_options(&options_->output_options());

    Base::onupdatecontrols();
    setEnabled(true);
  }

}
