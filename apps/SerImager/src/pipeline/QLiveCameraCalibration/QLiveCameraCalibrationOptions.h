/*
 * QLiveCameraCalibrationOptions.h
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveCameraCalibrationOptions_h__
#define __QLiveCameraCalibrationOptions_h__

#include "QLiveCameraCalibrationPipeline.h"
#include <gui/qcameracalibration/QCameraCalibrationOptions.h>

namespace serimager {

class QLiveCameraCalibrationOptions :
    public QSettingsWidget
{
public:
  typedef QLiveCameraCalibrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QLiveCameraCalibrationOptions(QWidget * parent = nullptr);
  QLiveCameraCalibrationOptions(QLiveCameraCalibrationPipeline * pipeline, QWidget * parent = nullptr);

  void setPipeline(QLiveCameraCalibrationPipeline * pipeline);
  QLiveCameraCalibrationPipeline * pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  QLiveCameraCalibrationPipeline * pipeline_ = nullptr;
  QCameraCalibrationOptions * calibrationOptions_ctl = nullptr;
//  QCheckBox * save_frames_with_detected_chessboard_ctl = nullptr;
//  QLineEditBox * frames_with_detected_chessboard_filename_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QLiveCameraCalibrationOptions_h__ */
