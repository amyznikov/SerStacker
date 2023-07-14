/*
 * QStereoCalibrationInputOptions.h
 *
 *  Created on: Jul 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationInputOptions_h__
#define __QStereoCalibrationInputOptions_h__

#include <gui/qpipeline/stereo/QStereoInputOptions.h>
#include <core/pipeline/c_stereo_calibration_pipeline/c_stereo_calibration_pipeline.h>

class QStereoCalibrationInputOptions :
    public QStereoInputOptions<c_stereo_calibration_pipeline>
{
public:
  typedef QStereoCalibrationInputOptions ThisClass;
  typedef QStereoInputOptions<c_stereo_calibration_pipeline> Base;

  QStereoCalibrationInputOptions(QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;
};

#endif /* __QStereoCalibrationInputOptions_h__ */
