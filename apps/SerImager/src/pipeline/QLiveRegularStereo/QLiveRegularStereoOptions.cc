/*
 * QLiveRegularStereoOptions.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QLiveRegularStereoOptions.h"
#if 0

#include <core/debug.h>

namespace serimager {

QLiveRegularStereoOptions::QLiveRegularStereoOptions(QWidget * parent) :
  ThisClass(nullptr, parent)
{
}

QLiveRegularStereoOptions::QLiveRegularStereoOptions(QLiveRegularStereoPipeline * pipeline, QWidget * parent) :
    Base("", parent)
{
  addRow(regularStereoOptions_ctl = new QRegularStereoOptions());
  regularStereoOptions_ctl->layout()->setContentsMargins(0, 0, 0, 0);
  connect(regularStereoOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  setPipeline(pipeline);
}

void QLiveRegularStereoOptions::onLivePipelineStateChanged(bool isRunning)
{
  regularStereoOptions_ctl->updateRunTimeStateControls(isRunning);
}

void QLiveRegularStereoOptions::update_pipeline_controls()
{
  regularStereoOptions_ctl->set_rstereo(&*pipeline_);
  regularStereoOptions_ctl->updateRunTimeStateControls(pipeline_->isRunning());
  setEnabled(true);
}


} /* namespace serimager */

#endif // 0
