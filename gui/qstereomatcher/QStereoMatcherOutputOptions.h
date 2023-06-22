/*
 * QStereoMatcherOutputOptions.h
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoMatcherOutputOptions_h__
#define __QStereoMatcherOutputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_stereo_matcher_pipeline.h>

class QStereoMatcherOutputOptions :
    public QSettingsWidget
{
public:
  typedef QStereoMatcherOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoMatcherOutputOptions(QWidget * parent = nullptr);

  void set_output_options(c_stereo_output_options *options);
  c_stereo_output_options * output_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_output_options *options_ = nullptr;
  QBrowsePathCombo * output_directory_ctl = nullptr;

  QCheckBox * save_progress_video_ctl = nullptr;
  QLineEditBox * progress_video_filename_ctl = nullptr;

  QCheckBox * save_depthmaps_ctl = nullptr;
  QLineEditBox * depthmaps_filename_ctl = nullptr;

  QCheckBox * save_cloud3d_image_ctl = nullptr;
  QLineEditBox * cloud3d_image_filename_ctl = nullptr;

  QCheckBox * save_cloud3d_ply_ctl = nullptr;
  QLineEditBox * cloud3d_ply_filename_ctl = nullptr;

};

#endif /* __QStereoMatcherOutputOptions_h__ */
