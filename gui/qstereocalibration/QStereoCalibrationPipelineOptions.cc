/*
 * QStereoCalibrationPipelineOptions.cc
 *
 *  Created on: Mar 21, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrationPipelineOptions.h"

QStereoCalibrationPipelineOptions::QStereoCalibrationPipelineOptions(QWidget * parent) :
    Base("", parent)
{
  ///

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QStereoCalibrationInputOptions(this));
  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  addRow(stereoCalibrationOptions_ctl =
      new QStereoCalibrationOptions());

  stereoCalibrationOptions_ctl->layout()->setContentsMargins(0, 0, 0, 0);

  connect(stereoCalibrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QStereoCalibrationPipelineOptions::set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_stereo_calibration_pipeline::sptr & QStereoCalibrationPipelineOptions::current_pipeline() const
{
  return pipeline_;
}

void QStereoCalibrationPipelineOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {

    setEnabled(false);

    inputOptions_ctl->set_current_pipeline(nullptr);
    stereoCalibrationOptions_ctl->set_options(nullptr);
  }
  else {

    inputOptions_ctl->set_current_pipeline(pipeline_);
    stereoCalibrationOptions_ctl->set_options(&*pipeline_);

    Base::onupdatecontrols();
    setEnabled(true);
  }

}