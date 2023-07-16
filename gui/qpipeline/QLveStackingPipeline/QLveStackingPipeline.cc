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
}

QLveStackingSettingsWidget::QLveStackingSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  ///

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QLveStackingInputOptions());

  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  add_expandable_groupbox("Accumulation options",
      accumulate_ctl = new QLiveStackingAccumulateOptions());

  connect(accumulate_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  add_expandable_groupbox("Output options",
      outputOptions_ctl = new QLveStackingOutputOptions());

  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  updateControls();
}

void QLveStackingSettingsWidget::update_pipeline_controls()
{
  Base::update_pipeline_controls();

  if( !pipeline_ ) {
    inputOptions_ctl->set_pipeline(nullptr);
    accumulate_ctl->set_options(nullptr);
    outputOptions_ctl->set_output_options(nullptr);
  }
  else {
    inputOptions_ctl->set_pipeline(pipeline_);
    accumulate_ctl->set_options(&pipeline_->accumulation_options());
    outputOptions_ctl->set_output_options(&pipeline_->output_options());
  }
}
