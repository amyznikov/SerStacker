/*
 * QLiveCameraCalibrationOptions.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "QLiveCameraCalibrationOptions.h"

#if 0

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


void QLiveCameraCalibrationOptions::update_pipeline_controls()
{
  calibrationOptions_ctl->set_camera_calibration(pipeline_);
}

} /* namespace serimager */

#endif // 0
