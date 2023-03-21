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
//#include "QStereoCalibrationInputOptions.h"
//#include "QStereoCalibrateOptions.h"
//#include "QStereoCalibrationOutputOptions.h"

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

//  QNumberEditBox * chessboardSize_ctl = nullptr;
//  QNumberEditBox * chessboardCellSize_ctl = nullptr;
//  QChessboardCornersDetectionOptions * chessboardCornersDetection_ctl = nullptr;
//  QStereoCalibrateOptions * stereoCalibrateOptions_ctl = nullptr;
//  QStereoCalibrationOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QStereoCalibrationPipelineOptions_h__ */
