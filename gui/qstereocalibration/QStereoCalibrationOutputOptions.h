/*
 * QStereoCalibrationOutputOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationOutputOptions_h__
#define __QStereoCalibrationOutputOptions_h__


#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/stereo_calibration/c_stereo_calibration.h>

class QStereoCalibrationOutputOptions :
    public QSettingsWidget
{
public:
  typedef QStereoCalibrationOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoCalibrationOutputOptions(QWidget * parent = nullptr);

  void set_options(c_stereo_calibration_output_options * options);
  c_stereo_calibration_output_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_calibration_output_options * options_ = nullptr;

  QBrowsePathCombo * output_directory_ctl = nullptr;

  QCheckBox * save_rectified_images_ctl = nullptr;
  QLineEditBox * rectified_images_file_name_ctl = nullptr;

  QCheckBox * save_stereo_rectified_frames_ctl = nullptr;
  QLineEditBox * stereo_rectified_frames_file_name_ctl = nullptr;

  QCheckBox * save_quad_rectified_frames_ctl = nullptr;
  QLineEditBox * quad_rectified_frames_file_name_ctl = nullptr;


};

#endif /* __QStereoCalibrationOutputOptions_h__ */
