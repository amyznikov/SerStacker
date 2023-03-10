/*
 * QRStereoCalibrateOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoCalibrateOptions_h__
#define __QRStereoCalibrateOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_regular_stereo_pipeline.h>

class QRStereoCalibrateOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoCalibrateOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoCalibrateOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline);
  const c_regular_stereo_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_regular_stereo_pipeline::sptr pipeline_;

  QCheckBox * enable_calibration_ctl = nullptr;
  QBrowsePathCombo * calibration_config_filename_ctl = nullptr;

  QNumberEditBox * min_frames_ctl = nullptr;
  QNumberEditBox * max_frames_ctl = nullptr;
  //QNumberEditBox * max_iterations_ctl = nullptr;
  //QNumberEditBox * eps_ctl = nullptr;
  //QFlagsEditBox<RSTEREO_CALIBRATION_FLAGS> * calibration_flags_ctl = nullptr;
  //QCheckBox * auto_tune_calibration_flags_ctl = nullptr;
  QNumberEditBox * filter_alpha_ctl = nullptr;
};

#endif /* __QRStereoCalibrateOptions_h__ */
