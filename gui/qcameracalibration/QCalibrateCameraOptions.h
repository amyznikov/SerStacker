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
#include <core/pipeline/camera_calibration/c_camera_calibration.h>

class QCalibrateCameraOptions :
    public QSettingsWidget
{
public:
  typedef QCalibrateCameraOptions ThisClass;
  typedef QSettingsWidget Base;

  QCalibrateCameraOptions(QWidget * parent = nullptr);

  void set_options(c_calibrate_camera_options * options);
  c_calibrate_camera_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_calibrate_camera_options * options_ = nullptr;

  QNumericBox * min_frames_ctl = nullptr;
  QNumericBox * max_frames_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
  QFlagsEditBox<CAMERA_CALIBRATION_FLAGS> * calibration_flags_ctl = nullptr;
  QCheckBox * auto_tune_calibration_flags_ctl = nullptr;
  QNumericBox * filter_alpha_ctl = nullptr;
};

#endif /* __QCalibrateCameraOptions_h__ */
