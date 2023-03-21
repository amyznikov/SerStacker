/*
 * QLiveStereoCalibrationOptions.cc
 *
 *  Created on: Mar 21, 2023
 *      Author: amyznikov
 */

#include "QLiveStereoCalibrationOptions.h"

namespace serimager {

QLiveStereoCalibrationOptions::QLiveStereoCalibrationOptions(QWidget * parent) :
  ThisClass(nullptr, parent)
{
}

QLiveStereoCalibrationOptions::QLiveStereoCalibrationOptions(QLiveStereoCalibrationPipeline * pipeline, QWidget * parent) :
    Base("", parent)
{
  addRow(stereoCalibrationOptions_ctl =
      new QStereoCalibrationOptions());

  stereoCalibrationOptions_ctl->layout()->setContentsMargins(0, 0, 0, 0);

  connect(stereoCalibrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  setPipeline(pipeline);
}

void QLiveStereoCalibrationOptions::setPipeline(QLiveStereoCalibrationPipeline * pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

QLiveStereoCalibrationPipeline * QLiveStereoCalibrationOptions::pipeline() const
{
  return pipeline_;
}

void QLiveStereoCalibrationOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {
    setEnabled(false);

    stereoCalibrationOptions_ctl->set_options(nullptr);
  }
  else {

    stereoCalibrationOptions_ctl->set_options(&pipeline_->stereo_calibration());

    Base::onupdatecontrols();
    setEnabled(true);
  }

}

} /* namespace serimager */
