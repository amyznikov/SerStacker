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

    Base::onupdatecontrols();
    setEnabled(true);
  }

}

} /* namespace serimager */
