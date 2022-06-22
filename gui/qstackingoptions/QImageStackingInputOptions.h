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
#include <gui/widgets/QBrowsePathCombo.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QImageStackingInputOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageStackingInputOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<anscombe_method> QAnscombeMethodCombo;

  QImageStackingInputOptions(QWidget * parent = Q_NULLPTR);

  void set_input_options(c_input_options * options);
  c_input_options * input_options() const;

signals:
  void applyInputOptionsToAllRequested(const c_input_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_input_options * options_ = Q_NULLPTR;

  QCheckBox * enable_remove_bad_pixels_ctl = Q_NULLPTR;
  QNumberEditBox * bad_pixels_variation_threshold_ctl = Q_NULLPTR;

  QCheckBox * enable_color_maxtrix_ctl = Q_NULLPTR;
  QAnscombeMethodCombo * anscombe_ctl  = Q_NULLPTR;

  QBrowsePathCombo * missing_pixel_mask_filename_ctl = Q_NULLPTR;
  QCheckBox * missing_pixels_marked_black_ctl  = Q_NULLPTR;
  QCheckBox * inpaint_missing_pixels_ctl = Q_NULLPTR;

  QNumberEditBox * start_frame_index_ctl = Q_NULLPTR;
  QNumberEditBox * max_input_frames_ctl = Q_NULLPTR;

  QImageProcessorSelectionCombo * processor_selector_ctl = Q_NULLPTR;

  QToolButton * applyToAll_ctl = Q_NULLPTR;
};

#endif /* __QImageStackingInputOptions_h__ */
