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
#include <core/pipeline/stereo/c_stereo_input.h>
//#include <core/pipeline/c_stereo_calibration_pipeline.h>

class QStereoInputOptions :
    public QSettingsWidget
{
public:
  typedef QStereoInputOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoInputOptions(QWidget * parent = nullptr);

  void set_input_options(c_stereo_input_options * options);
  c_stereo_input_options * input_options() const;

protected:
  void onupdatecontrols() override;
  void populatesources();
  void updatesourcecontrols();

protected:
  c_stereo_input_options * options_ = nullptr;

  QEnumComboBox<stereo_input_frame_layout_type> * layout_type_ctl = nullptr;
  QCheckBox * swap_cameras_ctl = nullptr;

  QComboBox * left_source_ctl = nullptr;
  QComboBox * right_source_ctl = nullptr;

  QNumericBox * start_frame_index_ctl = nullptr;
  QNumericBox * max_input_frames_ctl = nullptr;
  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QCheckBox * inpaint_missing_pixels_ctl = nullptr;
};

#endif /* __QStereoCalibrationInputOptions_h__ */
