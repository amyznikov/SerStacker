/*
 * QRStereoOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoInputOptions_h__
#define __QRStereoInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_regular_stereo_pipeline/c_regular_stereo_pipeline.h>

class QRStereoInputOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoInputOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoInputOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline);
  const c_regular_stereo_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;
  void populatesources();

protected:
  c_regular_stereo_pipeline::sptr pipeline_;

  QComboBox * left_source_ctl = nullptr;
  QComboBox * right_source_ctl = nullptr;

  QNumericBox * start_frame_index_ctl = nullptr;
  QNumericBox * max_input_frames_ctl = nullptr;

  QCheckBox * convert_to_grayscale_ctl = nullptr;
  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QCheckBox * inpaint_missing_pixels_ctl = nullptr;

  QPlainTextEdit * leftCameraMatrixEditBox_ctl = nullptr;
  QPlainTextEdit * rightCameraMatrixEditBox_ctl = nullptr;

};

#endif /* __QRStereoInputOptions_h__ */
