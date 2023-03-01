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
#include "QCameraCalibrationInputOptions.h"
#include "QChessboardCornersDetectionOptions.h"
#include "QCalibrateCameraOptions.h"
#include "QCameraCalibrationOutputOptions.h"
#include <core/pipeline/c_chessboard_camera_calibration_pipeline.h>


class QCameraCalibrationOptions:
    public QSettingsWidget
{
public:
  typedef QCameraCalibrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QCameraCalibrationOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_chessboard_camera_calibration_pipeline::sptr & pipeline);
  const c_chessboard_camera_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_chessboard_camera_calibration_pipeline::sptr current_pipeline_;
  QNumberEditBox * chessboardSize_ctl = nullptr;
  QNumberEditBox * chessboardCellSize_ctl = nullptr;
  QCameraCalibrationInputOptions * inputOptions_ctl = nullptr;
  QChessboardCornersDetectionOptions * chessboardCornersDetection_ctl = nullptr;
  QCalibrateCameraOptions * calibrateCameraOptions_ctl = nullptr;
  QCameraCalibrationOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QCameraCalibrationOptions_h__ */
