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

class QCameraCalibrationPipeline :
    public QImageProcessingPipelineTemplate<c_camera_calibration_pipeline>
{
public:
  typedef QCameraCalibrationPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_camera_calibration_pipeline> Base;

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
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }
};

#endif /* __QCameraCalibrationPipeline_h__ */
