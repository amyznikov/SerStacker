/*
 * QCalibrateCameraOptions.h
 *
 *  Created on: Feb 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCalibrateCameraOptions_h__
#define __QCalibrateCameraOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_chessboard_camera_calibration_pipeline.h>

class QCalibrateCameraOptions :
    public QSettingsWidget
{
public:
  typedef QCalibrateCameraOptions ThisClass;
  typedef QSettingsWidget Base;

  QCalibrateCameraOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_chessboard_camera_calibration_pipeline::sptr & pipeline);
  const c_chessboard_camera_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_chessboard_camera_calibration_pipeline::sptr pipeline_;

  QNumberEditBox * min_frames_ctl = nullptr;
  QNumberEditBox * max_frames_ctl = nullptr;
  QNumberEditBox * max_iterations_ctl = nullptr;
  QNumberEditBox * eps_ctl = nullptr;
  QFlagsEditBox<CAMERA_CALIBRATION_FLAGS> * calibration_flags_ctl = nullptr;
  QCheckBox * auto_tune_calibration_flags_ctl = nullptr;
  QNumberEditBox * filter_alpha_ctl = nullptr;
};

#endif /* __QCalibrateCameraOptions_h__ */
