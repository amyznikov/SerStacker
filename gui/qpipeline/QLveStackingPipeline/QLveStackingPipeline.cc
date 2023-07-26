/*
 * QLveStackingPipeline.cc
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#include "QLveStackingPipeline.h"



QLveStackingSettingsWidget::QLveStackingSettingsWidget(QWidget * parent) :
  ThisClass("", parent)
{
  updateControls();
}

QLveStackingSettingsWidget::QLveStackingSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  addRow(settings_ctl = new QPipelineSettingsCtrl(c_live_stacking_pipeline::get_controls(), this));
  connect(settings_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QLveStackingSettingsWidget::update_pipeline_controls()
{
  settings_ctl->set_pipeline(pipeline_);
  Base::update_pipeline_controls();
}
