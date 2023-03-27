/*
 * QLiveCameraCalibrationOptions.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "QLiveCameraCalibrationOptions.h"

namespace serimager {

QLiveCameraCalibrationOptions::QLiveCameraCalibrationOptions(QWidget * parent) :
  ThisClass(nullptr, parent)
{
}

QLiveCameraCalibrationOptions::QLiveCameraCalibrationOptions(QLiveCameraCalibrationPipeline * pipeline, QWidget * parent) :
    Base("", parent)
{
  addRow(calibrationOptions_ctl =
      new QCameraCalibrationOptions());

  calibrationOptions_ctl->layout()->setContentsMargins(0, 0, 0, 0);

  connect(calibrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  save_frames_with_detected_chessboard_ctl =
      calibrationOptions_ctl->outputOptions()->add_checkbox("Save frames:",
          [this](bool checked) {
            if ( pipeline_ && pipeline_->save_frames_with_detected_chessboard() != checked ) {
              pipeline_->set_save_frames_with_detected_chessboard(checked);
              frames_with_detected_chessboard_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              * checked = pipeline_->save_frames_with_detected_chessboard();
              return true;
            }
            return false;
          });

  save_frames_with_detected_chessboard_ctl->setToolTip("Save frames on which chessboard corners are detected");

  frames_with_detected_chessboard_filename_ctl =
      calibrationOptions_ctl->outputOptions()->add_textbox("filename:",
          [this](const QString & value) {
            if ( pipeline_ && pipeline_->frames_with_detected_chessboard_filename() != value ) {
              pipeline_->set_frames_with_detected_chessboard_filename(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              * value = pipeline_->save_frames_with_detected_chessboard();
              return true;
            }
            return false;
          });

  frames_with_detected_chessboard_filename_ctl->setToolTip("Output file name for frames with detected chesboard");

  setPipeline(pipeline);
}

void QLiveCameraCalibrationOptions::setPipeline(QLiveCameraCalibrationPipeline * pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

QLiveCameraCalibrationPipeline * QLiveCameraCalibrationOptions::pipeline() const
{
  return pipeline_;
}

void QLiveCameraCalibrationOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {
    setEnabled(false);

    calibrationOptions_ctl->set_options(nullptr);
  }
  else {

    calibrationOptions_ctl->set_options(&pipeline_->camera_calibration());
    frames_with_detected_chessboard_filename_ctl->setEnabled(pipeline_->save_frames_with_detected_chessboard());

    Base::onupdatecontrols();

    setEnabled(true);
  }
}

} /* namespace serimager */
