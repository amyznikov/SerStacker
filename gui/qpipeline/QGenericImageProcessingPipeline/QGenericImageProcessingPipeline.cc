/*
 * QGenericImageProcessingPipeline.cc
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#include "QGenericImageProcessingPipeline.h"



QGenericImageProcessingSettingsWidget::QGenericImageProcessingSettingsWidget(QWidget * parent) :
  ThisClass("", parent)
{
  updateControls();
}

QGenericImageProcessingSettingsWidget::QGenericImageProcessingSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  addRow(settings_ctl = new QPipelineSettingsCtrl(c_generic_image_processor_pipeline::get_controls(), this));
  connect(settings_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QGenericImageProcessingSettingsWidget::update_pipeline_controls()
{
  settings_ctl->set_pipeline(pipeline_);
  Base::update_pipeline_controls();
}
