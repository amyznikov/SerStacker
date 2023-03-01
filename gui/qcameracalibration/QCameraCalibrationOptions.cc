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

  chessboardSize_ctl =
      add_numeric_box<cv::Size>("Chessboard Size:",
          [this](const cv::Size & size) {
            if ( current_pipeline_ ) {
              current_pipeline_->set_chessboard_size(size);
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * size) {
            if ( current_pipeline_ ) {
              *size = current_pipeline_->chessboard_size();
              return true;
            }
            return false;
          });

  add_numeric_box<cv::Size2f>("Chessboard Cell Size [m]:",
      [this](const cv::Size2f & size) {
        if ( current_pipeline_ ) {
          current_pipeline_->set_chessboard_cell_size(size);
          Q_EMIT parameterChanged();
        }
      },
      [this](cv::Size2f * size) {
        if ( current_pipeline_ ) {
          *size = current_pipeline_->chessboard_cell_size();
          return true;
        }
        return false;
      });


  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QCameraCalibrationInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


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


void QCameraCalibrationOptions::set_current_pipeline(const c_camera_calibration_pipeline::sptr & pipeline)
{
  current_pipeline_ = pipeline;
  updateControls();
}

const c_camera_calibration_pipeline::sptr & QCameraCalibrationOptions::current_pipeline() const
{
  return current_pipeline_;
}

void QCameraCalibrationOptions::onupdatecontrols()
{
  if ( !current_pipeline_ ) {

    setEnabled(false);

    inputOptions_ctl->set_current_pipeline(nullptr);
    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(nullptr);
    calibrateCameraOptions_ctl->set_current_pipeline(nullptr);
    outputOptions_ctl->set_current_pipeline(nullptr);
  }
  else {

    inputOptions_ctl->set_current_pipeline(current_pipeline_);

    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(
        &current_pipeline_->chessboard_corners_detection_options());

    calibrateCameraOptions_ctl->set_current_pipeline(current_pipeline_);
    outputOptions_ctl->set_current_pipeline(current_pipeline_);

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
