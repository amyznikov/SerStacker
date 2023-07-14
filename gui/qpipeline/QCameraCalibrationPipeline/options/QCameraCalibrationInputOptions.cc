/*
 * QCameraCalibrationInputOptions.cc
 *
 *  Created on: Feb 28, 2023
 *      Author: amyznikov
 */

#include "QCameraCalibrationInputOptions.h"

QCameraCalibrationInputOptions::QCameraCalibrationInputOptions(QWidget * parent) :
    Base(parent)
{
  updateControls();
}

void QCameraCalibrationInputOptions::update_pipeline_controls()
{
  Base::update_pipeline_controls();
}

