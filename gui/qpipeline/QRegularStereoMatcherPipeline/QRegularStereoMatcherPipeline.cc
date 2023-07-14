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
}

QRegularStereoMatcherSettingsWidget::QRegularStereoMatcherSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QStereoMatcherInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Stereo rectification",
      stereoRectificationOptions_ctl = new QStereoRectificationOptions(this));
  connect(stereoRectificationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Processing options",
      processingOptions_ctl = new QStereoMatcherProcessingOptions());
  connect(processingOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Stereo Matcher",
      stereoMatcherOptions_ctl = new QStereoMatcherOptions());
  connect(stereoMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Output options",
      outputOptions_ctl = new QStereoMatcherOutputOptions());
  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  updateControls();
}

void QRegularStereoMatcherSettingsWidget::update_pipeline_controls()
{
  Base::update_pipeline_controls();

  if ( !pipeline_ ) {
    inputOptions_ctl->set_pipeline(nullptr);
    stereoRectificationOptions_ctl->set_rectification_options(nullptr);
    processingOptions_ctl->set_processing_options(nullptr);
    stereoMatcherOptions_ctl->set_stereo_matcher(nullptr);
    outputOptions_ctl->set_output_options(nullptr);
  }
  else {
    inputOptions_ctl->set_pipeline(pipeline_);
    stereoRectificationOptions_ctl->set_rectification_options(&pipeline_->stereo_rectification_options());
    processingOptions_ctl->set_processing_options(&pipeline_->processing_options());
    stereoMatcherOptions_ctl->set_stereo_matcher(&pipeline_->stereo_matcher());
    outputOptions_ctl->set_output_options(&pipeline_->output_options());
  }
}
