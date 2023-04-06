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

void QLiveStereoCalibrationOptions::update_pipeline_controls()
{
  stereoCalibrationOptions_ctl->set_stereo_calibration(pipeline_);
}

} /* namespace serimager */
