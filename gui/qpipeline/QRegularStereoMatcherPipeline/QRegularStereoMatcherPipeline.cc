/*
 * QRegularStereoMatcherPipeline.cc
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#include "QRegularStereoMatcherPipeline.h"


QRegularStereoMatcherSettingsWidget::QRegularStereoMatcherSettingsWidget(QWidget * parent) :
  ThisClass("", parent)
{
  updateControls();
}

QRegularStereoMatcherSettingsWidget::QRegularStereoMatcherSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  addRow(settings_ctl = new QPipelineSettingsCtrl(c_stereo_matcher_pipeline::get_controls(), this));
  connect(settings_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
  updateControls();
}

void QRegularStereoMatcherSettingsWidget::update_pipeline_controls()
{
  settings_ctl->set_pipeline(pipeline_);
  Base::update_pipeline_controls();
}
