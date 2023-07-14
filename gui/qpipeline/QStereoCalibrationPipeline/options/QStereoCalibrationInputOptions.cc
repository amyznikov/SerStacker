/*
 * QStereoCalibrationInputOptions.cc
 *
 *  Created on: Jul 14, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrationInputOptions.h"

QStereoCalibrationInputOptions::QStereoCalibrationInputOptions(QWidget * parent) :
  Base(parent)
{
  updateControls();
}

void QStereoCalibrationInputOptions::update_pipeline_controls()
{
  Base::update_pipeline_controls();
}
