/*
 * QCameraCalibrationPipeline.h
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraCalibrationPipeline_h__
#define __QCameraCalibrationPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_camera_calibration_pipeline/c_camera_calibration_pipeline.h>
#include <gui/qchessboardcorners/QChessboardCornersDetectionOptions.h>
#include "options/QCalibrateCameraOptions.h"
#include "options/QCameraCalibrationInputOptions.h"
#include "options/QCameraCalibrationOutputOptions.h"

class QCameraCalibrationPipeline;
class QCameraCalibrationSettingsWidget;

class QCameraCalibrationSettingsWidget :
    public QPipelineSettingsWidgetBase<QCameraCalibrationPipeline>
{
public:
  typedef QCameraCalibrationSettingsWidget  ThisClass;
  typedef QPipelineSettingsWidgetBase<QCameraCalibrationPipeline> Base;

  QCameraCalibrationSettingsWidget(QWidget * parent = nullptr);
  QCameraCalibrationSettingsWidget(const QString & prefix, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QCameraCalibrationInputOptions * inputOptions_ctl = nullptr;
  QChessboardCornersDetectionOptions * chessboardDetectionOptions_ctl = nullptr;
  QCalibrateCameraOptions * calibrateCameraOptions_ctl = nullptr;
  QCameraCalibrationOutputOptions * outputOptions_ctl = nullptr;
};

class QCameraCalibrationPipeline :
    public QImageProcessingPipelineBase<c_camera_calibration_pipeline>
{
public:
  typedef QCameraCalibrationPipeline ThisClass;
  typedef QImageProcessingPipelineBase<c_camera_calibration_pipeline> Base;

  QCameraCalibrationPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QCameraCalibrationPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QCameraCalibrationSettingsWidget(parent);
  }
};

#endif /* __QCameraCalibrationPipeline_h__ */
