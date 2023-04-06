/*
 * QLiveImageProcessingOptions.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "QLiveImageProcessingOptions.h"

namespace serimager {

QLiveImageProcessingOptions::QLiveImageProcessingOptions(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

QLiveImageProcessingOptions::QLiveImageProcessingOptions(QLiveImageProcessingPipeline * pipeline, QWidget * parent) :
    Base("", parent)
{

  addRow(genericOptions_ctl = new QGenericImageProcessorOptions());
  genericOptions_ctl->layout()->setContentsMargins(0, 0, 0, 0);
  connect(genericOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  setPipeline(pipeline);
}

void QLiveImageProcessingOptions::update_pipeline_controls()
{
  genericOptions_ctl->set_pipeline(pipeline_);
}

} // namespace serimager
