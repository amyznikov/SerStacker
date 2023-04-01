/*
 * QCameraCalibrationOutputOptions.h
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraCalibrationOutputOptions_h__
#define __QCameraCalibrationOutputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/camera_calibration/c_camera_calibration.h>

class QCameraCalibrationOutputOptions :
    public QSettingsWidget
{
public:
  typedef QCameraCalibrationOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QCameraCalibrationOutputOptions(QWidget * parent = nullptr);

  void set_options(c_camera_calibration_output_options * options);
  c_camera_calibration_output_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_camera_calibration_output_options * options_ = nullptr;
  QBrowsePathCombo * output_directory_ctl = nullptr;

  QCheckBox * save_chessboard_frames_ctl = nullptr;
  QLineEditBox * chessboard_frames_filename_ctl = nullptr;

  QCheckBox * save_rectified_frames_ctl = nullptr;
  QLineEditBox * rectified_frames_filename_ctl = nullptr;

  QCheckBox * save_progress_video_ctl = nullptr;
  QLineEditBox * progress_video_filename_ctl = nullptr;

};


#endif /* __QCameraCalibrationOutputOptions_h__ */
