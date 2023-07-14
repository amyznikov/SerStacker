/*
 * QCameraCalibrationInputOptions.h
 *
 *  Created on: Feb 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraCalibrationInputOptions_h__
#define __QCameraCalibrationInputOptions_h__

#include <gui/qpipeline/QPipelineInputOptions.h>
#include <core/pipeline/c_camera_calibration_pipeline/c_camera_calibration_pipeline.h>

class QCameraCalibrationInputOptions :
    public QPipelineInputOptions<c_camera_calibration_pipeline>
{
public:
  typedef QCameraCalibrationInputOptions ThisClass;
  typedef QPipelineInputOptions<c_camera_calibration_pipeline> Base;

  QCameraCalibrationInputOptions(QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;
};

#endif /* __QCameraCalibrationInputOptions_h__ */
