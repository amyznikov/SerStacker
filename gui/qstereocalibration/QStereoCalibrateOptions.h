/*
 * QStereoCalibrateOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrateOptions_h__
#define __QStereoCalibrateOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_stereo_calibration_pipeline.h>

class QStereoCalibrateOptions :
    public QSettingsWidget
{
public:
  typedef QStereoCalibrateOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoCalibrateOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline);
  const c_stereo_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_calibration_pipeline::sptr pipeline_;

  QNumberEditBox * min_frames_ctl = nullptr;
  QNumberEditBox * max_frames_ctl = nullptr;
  QNumberEditBox * max_iterations_ctl = nullptr;
  QNumberEditBox * eps_ctl = nullptr;
  QFlagsEditBox<STEREO_CALIBRATION_FLAGS> * calibration_flags_ctl = nullptr;
  QCheckBox * auto_tune_calibration_flags_ctl = nullptr;
  QNumberEditBox * filter_alpha_ctl = nullptr;
};

#endif /* __QStereoCalibrateOptions_h__ */
