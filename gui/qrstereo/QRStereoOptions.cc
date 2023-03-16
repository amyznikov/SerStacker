/*
 * QRStereoOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QRStereoOptions.h"


QRStereoOptions::QRStereoOptions(QWidget * parent) :
  Base("QCameraCalibrationSettings", parent)
{

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QRStereoInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Feature2D options",
      feature2DOptions_ctl = new QRStereoFeature2dOptions(this));
  connect(feature2DOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Calibrate Options",
      stereoCalibrateOptions_ctl = new QRStereoCalibrateOptions(this));
  connect(stereoCalibrateOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Stereo Matching",
      stereoMatchingOptions_ctl = new QRStereoMatchingOptions(this));
  connect(stereoMatchingOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Image Processing",
      imageProcessingOptions_ctl = new QRStereoImageProcessingOptions(this));
  connect(imageProcessingOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Output Options",
      outputOptions_ctl = new QRStereoOutputOptions(this));
  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  updateControls();
}


void QRStereoOptions::set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_regular_stereo_pipeline::sptr & QRStereoOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {

    setEnabled(false);

    inputOptions_ctl->set_current_pipeline(nullptr);
    feature2DOptions_ctl->set_feature2d_options(nullptr);
    stereoCalibrateOptions_ctl->set_current_pipeline(nullptr);
    stereoMatchingOptions_ctl->set_current_pipeline(nullptr);
    imageProcessingOptions_ctl->set_current_pipeline(nullptr);
    outputOptions_ctl->set_current_pipeline(nullptr);
  }
  else {

    inputOptions_ctl->set_current_pipeline(pipeline_);
    feature2DOptions_ctl->set_feature2d_options(&pipeline_->feature2d_options());
    stereoCalibrateOptions_ctl->set_current_pipeline(pipeline_);
    stereoMatchingOptions_ctl->set_current_pipeline(pipeline_);
    imageProcessingOptions_ctl->set_current_pipeline(pipeline_);
    outputOptions_ctl->set_current_pipeline(pipeline_);

    Base::onupdatecontrols();
    setEnabled(true);
  }

}
