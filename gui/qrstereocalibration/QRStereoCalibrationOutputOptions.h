/*
 * QRStereoCalibrationOutputOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoCalibrationOutputOptions_h__
#define __QRStereoCalibrationOutputOptions_h__


#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_rstereo_calibration_pipeline.h>

class QRStereoCalibrationOutputOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoCalibrationOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoCalibrationOutputOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_rstereo_calibration_pipeline::sptr & pipeline);
  const c_rstereo_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_rstereo_calibration_pipeline::sptr pipeline_;
  QBrowsePathCombo * output_directory_ctl = nullptr;

  QCheckBox * save_progress_video_ctl = nullptr;
  QLineEditBox * progress_video_filename_ctl = nullptr;

  QCheckBox * save_rectified_video_ctl = nullptr;
  QLineEditBox * rectified_video_filename_ctl = nullptr;

  QCheckBox * save_stereo_matches_video_ctl = nullptr;
  QLineEditBox * stereo_matches_video_filename_ctl = nullptr;

  QCheckBox * save_motion_poses_ctl = nullptr;
  QLineEditBox * motion_poses_filename_ctl = nullptr;
};

#endif /* __QRStereoCalibrationOutputOptions_h__ */
