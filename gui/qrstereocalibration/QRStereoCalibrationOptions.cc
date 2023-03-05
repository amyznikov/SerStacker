/*
 * QRStereoCalibrationOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QRStereoCalibrationOptions.h"


QRStereoCalibrationOptions::QRStereoCalibrationOptions(QWidget * parent) :
  Base("QCameraCalibrationSettings", parent)
{

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QRStereoCalibrationInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Feature2D options",
      registration_options_ctl = new QRStereoFeature2dOptions(this));
  connect(registration_options_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Calibrate Options",
      stereoCalibrateOptions_ctl = new QRStereoCalibrateOptions(this));
  connect(stereoCalibrateOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Output Options",
      outputOptions_ctl = new QRStereoCalibrationOutputOptions(this));
  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  updateControls();
}


void QRStereoCalibrationOptions::set_current_pipeline(const c_rstereo_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_rstereo_calibration_pipeline::sptr & QRStereoCalibrationOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoCalibrationOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {

    setEnabled(false);

    inputOptions_ctl->set_current_pipeline(nullptr);
    stereoCalibrateOptions_ctl->set_current_pipeline(nullptr);
    outputOptions_ctl->set_current_pipeline(nullptr);
    registration_options_ctl->set_feature2d_options(nullptr);
  }
  else {

    inputOptions_ctl->set_current_pipeline(pipeline_);
    registration_options_ctl->set_feature2d_options(&pipeline_->feature2d_options());
    stereoCalibrateOptions_ctl->set_current_pipeline(pipeline_);
    outputOptions_ctl->set_current_pipeline(pipeline_);

    Base::onupdatecontrols();
    setEnabled(true);
  }

}
