/*
 * QCameraCalibrationOptions.h
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraCalibrationOptions_h__
#define __QCameraCalibrationOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qchessboardcorners/QChessboardCornersDetectionOptions.h>
#include <core/pipeline/c_camera_calibration_pipeline.h>
#include "QCameraCalibrationInputOptions.h"
#include "QCalibrateCameraOptions.h"
#include "QCameraCalibrationOutputOptions.h"


class QCameraCalibrationOptions:
    public QSettingsWidget
{
public:
  typedef QCameraCalibrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QCameraCalibrationOptions(QWidget * parent = nullptr);

  void set_options(c_camera_calibration * options);
  c_camera_calibration * options() const;

  QChessboardCornersDetectionOptions * chessboardDetectionOptions() const;
  QCalibrateCameraOptions * calibrateCameraOptions() const;
  QCameraCalibrationOutputOptions * outputOptions() const;

protected:
  void onupdatecontrols() override;

protected:
  c_camera_calibration * options_ = nullptr;
  QChessboardCornersDetectionOptions * chessboardDetectionOptions_ctl = nullptr;
  QCalibrateCameraOptions * calibrateCameraOptions_ctl = nullptr;
  QCameraCalibrationOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QCameraCalibrationOptions_h__ */
