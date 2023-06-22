/*
 * QStereoMatcherPipelineOptions.cc
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#include "QStereoMatcherPipelineOptions.h"

QStereoMatcherPipelineOptions::QStereoMatcherPipelineOptions(QWidget * parent) :
  Base("", parent)
{
  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QStereoInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  add_expandable_groupbox("Stereo matcher options",
      regularStereoOptions_ctl = new QRegularStereoOptions());
  connect(regularStereoOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Processing options",
      processingOptions_ctl = new QStereoMatcherProcessingOptions());
  connect(processingOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Output options",
      outputOptions_ctl = new QStereoMatcherOutputOptions());
  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  updateControls();
}


void QStereoMatcherPipelineOptions::set_current_pipeline(const c_stereo_matcher_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_stereo_matcher_pipeline::sptr & QStereoMatcherPipelineOptions::current_pipeline() const
{
  return pipeline_;
}

void QStereoMatcherPipelineOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {

    setEnabled(false);

    inputOptions_ctl->set_input_options(nullptr);
    regularStereoOptions_ctl->set_rstereo(nullptr);
    processingOptions_ctl->set_processing_options(nullptr);
    outputOptions_ctl->set_output_options(nullptr);
  }
  else {

    inputOptions_ctl->set_input_options(&pipeline_->input_options());
    regularStereoOptions_ctl->set_rstereo(pipeline_.get());
    processingOptions_ctl->set_processing_options(&pipeline_->processing_options());
    outputOptions_ctl->set_output_options(&pipeline_->output_options());

    Base::onupdatecontrols();
    setEnabled(true);
  }
}
