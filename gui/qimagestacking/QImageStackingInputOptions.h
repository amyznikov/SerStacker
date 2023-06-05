/*
 * QImageStackingInputOptions.h
 *
 *  Created on: Jul 20, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageStackingInputOptions_h__
#define __QImageStackingInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_image_stacking_pipeline.h>
#include "QBackgroundNormalizationOptions.h"

class QImageStackingInputOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageStackingInputOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<anscombe_method> QAnscombeMethodCombo;

  QImageStackingInputOptions(QWidget * parent = nullptr);

  void set_input_options(c_input_options * options);
  c_input_options * input_options() const;

//Q_SIGNALS:
//  void applyInputOptionsToAllRequested(const c_input_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_input_options * options_ = nullptr;

  QNumericBox * start_frame_index_ctl = nullptr;
  QNumericBox * max_input_frames_ctl = nullptr;

  QCheckBox * enable_remove_bad_pixels_ctl = nullptr;
  QNumericBox * bad_pixels_variation_threshold_ctl = nullptr;
  QCheckBox * drop_bad_asi_frames_ctl = nullptr;

  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QAnscombeMethodCombo * anscombe_ctl  = nullptr;

  QBrowsePathCombo * darkbayer_filename_ctl = nullptr;
  QBrowsePathCombo * flatbayer_filename_ctl = nullptr;
  QBrowsePathCombo * missing_pixel_mask_filename_ctl = nullptr;
  QCheckBox * missing_pixels_marked_black_ctl  = nullptr;
  QCheckBox * inpaint_missing_pixels_ctl = nullptr;

  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_method_ctl = nullptr;

  QCheckBox * enable_background_normalization_ctl = nullptr;
  QBackgroundNormalizationOptions * background_normalization_ctl = nullptr;
};

#endif /* __QImageStackingInputOptions_h__ */
