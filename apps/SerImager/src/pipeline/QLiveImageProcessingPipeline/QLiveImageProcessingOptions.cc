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


void QLiveImageProcessingOptions::setPipeline(QLiveImageProcessingPipeline * pipeline)
{
  if ( pipeline_ ) {
    pipeline_->disconnect(this);
  }

  if ( (pipeline_ = pipeline) ) {
//
//    connect(pipeline_, &QLivePipeline::runningStateChanged,
//        this, &ThisClass::onLivePipelineStateChanged,
//        Qt::QueuedConnection);
  }

  updateControls();
}

QLiveImageProcessingPipeline * QLiveImageProcessingOptions::pipeline() const
{
  return pipeline_;
}

void QLiveImageProcessingOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {
    setEnabled(false);
    genericOptions_ctl->set_pipeline(nullptr);
  }
  else {
    genericOptions_ctl->set_pipeline(pipeline_);
    //genericOptions_ctl->updateRunTimeStateControls(pipeline_->isRunning());
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

} // namespace serimager
