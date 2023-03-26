/*
 * QCameraCalibrationPipelineOptions.h
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraCalibrationPipelineOptions_h__
#define __QCameraCalibrationPipelineOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qchessboardcorners/QChessboardCornersDetectionOptions.h>
#include <core/pipeline/c_camera_calibration_pipeline.h>
#include "QCameraCalibrationOptions.h"

class QCameraCalibrationPipelineOptions :
    public QSettingsWidget
{
public:
  typedef QCameraCalibrationPipelineOptions ThisClass;
  typedef QSettingsWidget Base;

  QCameraCalibrationPipelineOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_camera_calibration_pipeline::sptr & pipeline);
  const c_camera_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_camera_calibration_pipeline::sptr pipeline_;
  QCameraCalibrationInputOptions * inputOptions_ctl = nullptr;
  QCameraCalibrationOptions * cameraCalibrationOptions_ctl = nullptr;
};

#endif /* __QCameraCalibrationPipelineOptions_h__ */
