/*
 * QRStereoOutputOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoOutputOptions_h__
#define __QRStereoOutputOptions_h__


#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_regular_stereo_pipeline/c_regular_stereo_pipeline.h>

class QRStereoOutputOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoOutputOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline);
  const c_regular_stereo_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_regular_stereo_pipeline::sptr pipeline_;
  QBrowsePathCombo * output_directory_ctl = nullptr;

  QCheckBox * save_calibration_config_file_ctl = nullptr;
  QBrowsePathCombo * calibration_config_filename_ctl = nullptr;

  QCheckBox * save_progress_video_ctl = nullptr;
  QLineEditBox * progress_video_filename_ctl = nullptr;

  QCheckBox * save_rectified_video_ctl = nullptr;
  QLineEditBox * left_rectified_video_filename_ctl = nullptr;
  QLineEditBox * right_rectified_video_filename_ctl = nullptr;

  QCheckBox * save_stereo_matches_video_ctl = nullptr;
  QLineEditBox * stereo_matches_video_filename_ctl = nullptr;

  QCheckBox * save_motion_poses_ctl = nullptr;
  QLineEditBox * motion_poses_filename_ctl = nullptr;

  QCheckBox * save_stereo_match_progress_video_ctl = nullptr;
  QLineEditBox * stereo_match_progress_video_filename_ctl = nullptr;
};

#endif /* __QRStereoOutputOptions_h__ */
