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

  void set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline);
  const c_stereo_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_calibration_pipeline::sptr pipeline_;
  QNumberEditBox * chessboardSize_ctl = nullptr;
  QNumberEditBox * chessboardCellSize_ctl = nullptr;
  QStereoCalibrationInputOptions * inputOptions_ctl = nullptr;
  QChessboardCornersDetectionOptions * chessboardCornersDetection_ctl = nullptr;
  QStereoCalibrateOptions * stereoCalibrateOptions_ctl = nullptr;
  QStereoCalibrationOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QStereoCalibrationOptions_h__ */
