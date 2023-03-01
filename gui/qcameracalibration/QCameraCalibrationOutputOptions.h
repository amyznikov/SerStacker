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
#include <core/pipeline/c_camera_calibration_pipeline.h>

class QCameraCalibrationOutputOptions :
    public QSettingsWidget
{
public:
  typedef QCameraCalibrationOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QCameraCalibrationOutputOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_camera_calibration_pipeline::sptr & pipeline);
  const c_camera_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_camera_calibration_pipeline::sptr pipeline_;
  QBrowsePathCombo * output_directory_ctl = nullptr;
  QCheckBox * save_rectified_images_ctl = nullptr;
  QLineEditBox * rectified_images_file_name_ctl = nullptr;

};

#endif /* __QCameraCalibrationOutputOptions_h__ */
