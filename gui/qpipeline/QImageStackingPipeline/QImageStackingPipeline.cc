/*
 * QImageStackingPipeline.cc
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#include "QImageStackingPipeline.h"

QImageStackingSettingsWidget::QImageStackingSettingsWidget(QWidget * parent) :
    ThisClass("", parent)
{
}

QImageStackingSettingsWidget::QImageStackingSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
   add_expandable_groupbox("* Input Options",
      inputOptions_ctl =
          new QImageStackingInputOptions(this));

  add_expandable_groupbox("* ROI Selection",
      roiSelection_ctl =
          new QROISelectionOptions(this));

  add_expandable_groupbox("* Upscale Options",
      upscaleOptions_ctl =
          new QFrameUpscaleOptions(this));

  add_expandable_groupbox("* Frame Registration Options",
      frameRegistration_ctl =
          new QFrameRegistrationOptions(this));

  add_expandable_groupbox("* Frame Accumulation Options",
      frameAccumulation_ctl =
          new QFrameAccumulationOptions(this));

  add_expandable_groupbox("* Image processing",
      imageProcessingOptions_ctl =
          new QImageProcessingOptions(this));

  add_expandable_groupbox("* Output Options",
      outputOptions_ctl =
          new QStackOutputOptions(this));

  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(roiSelection_ctl, &QROISelectionOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(upscaleOptions_ctl, &QFrameUpscaleOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(frameAccumulation_ctl, &QFrameAccumulationOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(frameRegistration_ctl, &QFrameRegistrationOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(imageProcessingOptions_ctl, &QImageProcessingOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(outputOptions_ctl, &QStackOutputOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QImageStackingSettingsWidget::update_pipeline_controls()
{
  if ( !pipeline_ ) {
    inputOptions_ctl->set_input_options(nullptr);
    roiSelection_ctl->set_roi_selection_options(nullptr);
    upscaleOptions_ctl->set_upscale_options(nullptr);
    frameAccumulation_ctl->set_accumulation_options(nullptr);
    frameRegistration_ctl->set_current_pipeline(nullptr);
    imageProcessingOptions_ctl->set_image_processing_options(nullptr);
    outputOptions_ctl->set_output_options(nullptr);
  }
  else {
    inputOptions_ctl->set_input_options(&pipeline_->input_options());
    roiSelection_ctl->set_roi_selection_options(&pipeline_->roi_selection_options());
    upscaleOptions_ctl->set_upscale_options(&pipeline_->upscale_options());
    frameAccumulation_ctl->set_accumulation_options(&pipeline_->accumulation_options());
    imageProcessingOptions_ctl->set_image_processing_options(&pipeline_->image_processing_options());
    frameRegistration_ctl->set_current_pipeline(pipeline_);
    outputOptions_ctl->set_output_options(&pipeline_->output_options());
  }
}
