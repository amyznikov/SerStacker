/*
 * QStereoCalibrationInputOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationInputOptions_h__
#define __QStereoCalibrationInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_stereo_calibration_pipeline.h>

class QStereoCalibrationInputOptions :
    public QSettingsWidget
{
public:
  typedef QStereoCalibrationInputOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoCalibrationInputOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline);
  const c_stereo_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;
  void populatesources();

protected:
  c_stereo_calibration_pipeline::sptr pipeline_;

  QComboBox * left_source_ctl = nullptr;
  QComboBox * right_source_ctl = nullptr;

  QNumberEditBox * start_frame_index_ctl = nullptr;
  QNumberEditBox * max_input_frames_ctl = nullptr;
  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QCheckBox * inpaint_missing_pixels_ctl = nullptr;
};

#endif /* __QStereoCalibrationInputOptions_h__ */