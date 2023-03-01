/*
 * QStackingSettingsWidget.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QImageStackingOptions.h"
#include <gui/qpipelinethread/QImageProcessingPipeline.h>
//#include <gui/widgets/addctrl.h>
#include <gui/widgets/style.h>

#define ICON_close          ":/qstackingoptions/icons/close"
#define ICON_check_all      ":/qstackingoptions/icons/check_all"


QImageStackingOptions::QImageStackingOptions(QWidget * parent)
  : Base("QStackingSettingsWidget", parent)
{
  Q_INIT_RESOURCE(qstackingoptions_resources);

//  stackName_ctl =
//      add_textbox("* Stack Name:",
//          [this](const QString & text) {
//            if ( current_pipeline_ && !text.isEmpty() ) {
//              current_pipeline_->set_name(text.toStdString());
//              Q_EMIT stackNameChanged(current_pipeline_);
//            }
//          });

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


//  connect(inputOptions_ctl, &QImageStackingInputOptions::applyInputOptionsToAllRequested,
//      this, &ThisClass::applyInputOptionsToAllRequested);
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);



//  connect(roiSelection_ctl, &QROISelectionOptions::applyROISelectionOptionsToAllRequested,
//      this, &ThisClass::applyROISelectionOptionsToAllRequested);
  connect(roiSelection_ctl, &QROISelectionOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

//  connect(upscaleOptions_ctl, &QFrameUpscaleOptions::applyFrameUpScaleOptionsToAllRequested,
//      this, &ThisClass::applyFrameUpscaleOptionsToAllRequested);
  connect(upscaleOptions_ctl, &QFrameUpscaleOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

//  connect(frameAccumulation_ctl, &QFrameAccumulationOptions::applyFrameAccumulationOptionsToAllRequested,
//      this, &ThisClass::applyFrameAccumulationOptionsToAllRequested);
  connect(frameAccumulation_ctl, &QFrameAccumulationOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

//  connect(frameRegistration_ctl, &QFrameRegistrationOptions::applyFrameRegistrationOptionsToAllRequested,
//      this, &ThisClass::applyFrameRegistrationOptionsToAllRequested);
  connect(frameRegistration_ctl, &QFrameRegistrationOptions::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(imageProcessingOptions_ctl, &QImageProcessingOptions::parameterChanged,
      this, &ThisClass::parameterChanged);


//  connect(outputOptions_ctl, &QStackOutputOptions::applyOutputOptionsToAllRequested,
//      this, &ThisClass::applyOutputOptionsToAllRequested);
  connect(outputOptions_ctl, &QStackOutputOptions::parameterChanged,
      this, &ThisClass::parameterChanged);


  updateControls();
}

void QImageStackingOptions::set_current_pipeline(const c_image_stacking_pipeline::sptr & current_pipeline)
{
  current_pipeline_ = current_pipeline;
  updateControls();
}

const c_image_stacking_pipeline::sptr & QImageStackingOptions::current_pipeline() const
{
  return current_pipeline_;
}

void QImageStackingOptions::onupdatecontrols()
{
  if ( !current_pipeline_ ) {
    setEnabled(false);

    //stackName_ctl->setText("");
    inputOptions_ctl->set_input_options(nullptr);
    roiSelection_ctl->set_roi_selection_options(nullptr);
    upscaleOptions_ctl->set_upscale_options(nullptr);
    frameAccumulation_ctl->set_accumulation_options(nullptr);
    frameRegistration_ctl->set_current_pipeline(nullptr);
    imageProcessingOptions_ctl->set_image_processing_options(nullptr);
    outputOptions_ctl->set_current_pipeline(nullptr);

  }
  else {

    //stackName_ctl->setText(current_pipeline_->name().c_str());
    inputOptions_ctl->set_input_options(&current_pipeline_->input_options());
    roiSelection_ctl->set_roi_selection_options(&current_pipeline_->roi_selection_options());
    upscaleOptions_ctl->set_upscale_options(&current_pipeline_->upscale_options());
    frameAccumulation_ctl->set_accumulation_options(&current_pipeline_->accumulation_options());
    imageProcessingOptions_ctl->set_image_processing_options(&current_pipeline_->image_processing_options());
    frameRegistration_ctl->set_current_pipeline(current_pipeline_);
    outputOptions_ctl->set_current_pipeline(current_pipeline_);

//    if ( QImageProcessingPipeline::isRunning() && current_pipeline_ == QImageProcessingPipeline::current_pipeline() ) {
//      setEnabled(false);
//    }
//    else {
//      setEnabled(true);
//    }
  }
}

