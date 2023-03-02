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

  chessboardSize_ctl =
      add_numeric_box<cv::Size>("Chessboard Size:",
          [this](const cv::Size & size) {
            if ( pipeline_ ) {
              pipeline_->set_chessboard_size(size);
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * size) {
            if ( pipeline_ ) {
              *size = pipeline_->chessboard_size();
              return true;
            }
            return false;
          });

  add_numeric_box<cv::Size2f>("Chessboard Cell Size [m]:",
      [this](const cv::Size2f & size) {
        if ( pipeline_ ) {
          pipeline_->set_chessboard_cell_size(size);
          Q_EMIT parameterChanged();
        }
      },
      [this](cv::Size2f * size) {
        if ( pipeline_ ) {
          *size = pipeline_->chessboard_cell_size();
          return true;
        }
        return false;
      });


  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QStereoCalibrationInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Chessboard Corners Detection",
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


void QStereoCalibrationOptions::set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_stereo_calibration_pipeline::sptr & QStereoCalibrationOptions::current_pipeline() const
{
  return pipeline_;
}

void QStereoCalibrationOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {

    setEnabled(false);

    inputOptions_ctl->set_current_pipeline(nullptr);
    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(nullptr);
    stereoCalibrateOptions_ctl->set_current_pipeline(nullptr);
    outputOptions_ctl->set_current_pipeline(nullptr);
  }
  else {

    inputOptions_ctl->set_current_pipeline(pipeline_);

    chessboardCornersDetection_ctl->set_chessboard_corners_detection_options(
        &pipeline_->chessboard_corners_detection_options());

    stereoCalibrateOptions_ctl->set_current_pipeline(pipeline_);
    outputOptions_ctl->set_current_pipeline(pipeline_);

    Base::onupdatecontrols();
    setEnabled(true);
  }

}
