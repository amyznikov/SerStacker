/*
 * QLiveRegularStereoOptions.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QLiveRegularStereoOptions.h"
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

void QLiveRegularStereoOptions::setPipeline(QLiveRegularStereoPipeline * pipeline)
{
  if ( pipeline_ ) {
    pipeline_->disconnect(this);
  }

  if ( (pipeline_ = pipeline) ) {

    connect(pipeline_, &QLivePipeline::runningStateChanged,
        this, &ThisClass::onLivePipelineStateChanged,
        Qt::QueuedConnection);
  }

  updateControls();
}

QLiveRegularStereoPipeline * QLiveRegularStereoOptions::pipeline() const
{
  return pipeline_;
}

void QLiveRegularStereoOptions::onLivePipelineStateChanged(bool isRunning)
{
  CF_DEBUG("isRunning=%d", isRunning);
  updateControls();
  CF_DEBUG("updateControls() OK");
}

void QLiveRegularStereoOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {
    setEnabled(false);
    regularStereoOptions_ctl->set_rstereo(nullptr);
  }
  else {

    regularStereoOptions_ctl->set_rstereo(&*pipeline_);
    regularStereoOptions_ctl->updateRunTimeStateControls(pipeline_->isRunning());
    Base::onupdatecontrols();
    setEnabled(true);
  }
}


} /* namespace serimager */
