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
#include <core/pipeline/stereo_calibration/c_stereo_calibration.h>

class QStereoCalibrateOptions :
    public QSettingsWidget
{
public:
  typedef QStereoCalibrateOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoCalibrateOptions(QWidget * parent = nullptr);

  void set_options(c_stereo_calibrate_options * options);
  c_stereo_calibrate_options* options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_calibrate_options * options_ = nullptr;

  QNumericBox * min_frames_ctl = nullptr;
  QNumericBox * max_frames_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
  QFlagsEditBox<STEREO_CALIBRATION_FLAGS> * calibration_flags_ctl = nullptr;
  QCheckBox * auto_tune_calibration_flags_ctl = nullptr;
  QCheckBox * init_camera_matrix_2d_ctl = nullptr;
  QNumericBox * filter_alpha_ctl = nullptr;
};

#endif /* __QStereoCalibrateOptions_h__ */
