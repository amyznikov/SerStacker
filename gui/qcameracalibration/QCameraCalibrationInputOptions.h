/*
 * QCameraCalibrationInputOptions.h
 *
 *  Created on: Feb 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraCalibrationInputOptions_h__
#define __QCameraCalibrationInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_camera_calibration_pipeline.h>

class QCameraCalibrationInputOptions :
    public QSettingsWidget
{
public:
  typedef QCameraCalibrationInputOptions ThisClass;
  typedef QSettingsWidget Base;

  QCameraCalibrationInputOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_camera_calibration_pipeline::sptr & pipeline);
  const c_camera_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_camera_calibration_pipeline::sptr pipeline_;

  QNumericBox * start_frame_index_ctl = nullptr;
  QNumericBox * max_input_frames_ctl = nullptr;
  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QCheckBox * inpaint_missing_pixels_ctl = nullptr;

};

#endif /* __QCameraCalibrationInputOptions_h__ */
