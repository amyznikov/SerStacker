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
  updateControls();
}

QStereoCalibrationSettingsWidget::QStereoCalibrationSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  addRow(settings_ctl = new QPipelineSettingsCtrl(c_stereo_calibration_pipeline::get_controls(), this));
  connect(settings_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
  updateControls();
}

void QStereoCalibrationSettingsWidget::update_pipeline_controls()
{
  settings_ctl->set_pipeline(pipeline_);
  Base::update_pipeline_controls();
}
