/*
 * QStereoCalibrationPipelineOptions.h
 *
 *  Created on: Mar 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationPipelineOptions_h__
#define __QStereoCalibrationPipelineOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qchessboardcorners/QChessboardCornersDetectionOptions.h>
#include <core/pipeline/c_stereo_calibration_pipeline.h>
#include "QStereoCalibrationOptions.h"

class QStereoCalibrationPipelineOptions :
    public QSettingsWidget
{
public:
  typedef QStereoCalibrationPipelineOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoCalibrationPipelineOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline);
  const c_stereo_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_calibration_pipeline::sptr pipeline_;
  QStereoCalibrationInputOptions * inputOptions_ctl = nullptr;
  QStereoCalibrationOptions * stereoCalibrationOptions_ctl = nullptr;
};

#endif /* __QStereoCalibrationPipelineOptions_h__ */
