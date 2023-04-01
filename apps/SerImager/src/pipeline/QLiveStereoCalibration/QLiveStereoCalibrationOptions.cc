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
  if ( pipeline_ ) {
    pipeline_->disconnect(this);
  }

  if ( (pipeline_ = pipeline) ) {

    connect(pipeline_, &QLivePipeline::runningStateChanged,
        this, &ThisClass::updateControls,
        Qt::QueuedConnection);
  }

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

    stereoCalibrationOptions_ctl->set_options(&*pipeline_);

    Base::onupdatecontrols();

    setEnabled(!pipeline_->isRunning());
  }

}

} /* namespace serimager */
