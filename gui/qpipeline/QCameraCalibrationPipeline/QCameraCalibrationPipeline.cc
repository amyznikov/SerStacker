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
  updateControls();
}

QCameraCalibrationSettingsWidget::QCameraCalibrationSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  addRow(settings_ctl = new QPipelineSettingsCtrl(c_camera_calibration_pipeline::get_controls(), this));
  connect(settings_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
  updateControls();
}

void QCameraCalibrationSettingsWidget::update_pipeline_controls()
{
  settings_ctl->set_pipeline(pipeline_);
  Base::update_pipeline_controls();
}
