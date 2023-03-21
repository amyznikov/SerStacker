/*
 * QStereoCalibrationOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationOptions_h__
#define __QStereoCalibrationOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qchessboardcorners/QChessboardCornersDetectionOptions.h>
#include <core/pipeline/c_stereo_calibration_pipeline.h>
#include "QStereoCalibrationInputOptions.h"
#include "QStereoCalibrateOptions.h"
#include "QStereoCalibrationOutputOptions.h"

class QStereoCalibrationOptions :
    public QSettingsWidget
{
public:
  typedef QStereoCalibrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoCalibrationOptions(QWidget * parent = nullptr);

  void set_options(c_stereo_calibration * options);
  c_stereo_calibration *  options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_calibration * options_ = nullptr;
  QChessboardCornersDetectionOptions * chessboardCornersDetection_ctl = nullptr;
  QStereoCalibrateOptions * stereoCalibrateOptions_ctl = nullptr;
  QStereoCalibrationOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QStereoCalibrationOptions_h__ */
