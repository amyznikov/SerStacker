/*
 * QCameraCalibrationPipelineOptions.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "QCameraCalibrationPipelineOptions.h"

QCameraCalibrationPipelineOptions::QCameraCalibrationPipelineOptions(QWidget * parent) :
    Base("", parent)
{
  ///

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QCameraCalibrationInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  addRow(cameraCalibrationOptions_ctl =
      new QCameraCalibrationOptions());

  cameraCalibrationOptions_ctl->layout()->setContentsMargins(0, 0, 0, 0);

  connect(cameraCalibrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QCameraCalibrationPipelineOptions::set_current_pipeline(const c_camera_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_camera_calibration_pipeline::sptr & QCameraCalibrationPipelineOptions::current_pipeline() const
{
  return pipeline_;
}

void QCameraCalibrationPipelineOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {

    setEnabled(false);

    inputOptions_ctl->set_current_pipeline(nullptr);
    cameraCalibrationOptions_ctl->set_options(nullptr);
  }
  else {

    inputOptions_ctl->set_current_pipeline(pipeline_);
    cameraCalibrationOptions_ctl->set_options(&*pipeline_);

    Base::onupdatecontrols();
    setEnabled(true);
  }

}
