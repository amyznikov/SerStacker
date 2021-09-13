/*
 * QStackOutputOptions.h
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStackingDebugOptions_h__
#define __QStackingDebugOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QStackOutputOptions
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QStackOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QStackOutputOptions(QWidget * parent = Q_NULLPTR);

  void set_stacking_options(const c_image_stacking_options::ptr & options);
  const c_image_stacking_options::ptr & stacking_options() const;

signals:
  void applyOutputOptionsToAllRequested(const c_image_stacking_output_options & options);


protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_options::ptr options_ = Q_NULLPTR;

  QBrowsePathCombo * output_directory_ctl = Q_NULLPTR;

  QCheckBox * save_preprocessed_frames_ctl = Q_NULLPTR;
  QBrowsePathCombo * output_preprocessed_frames_path_ctl = Q_NULLPTR;

  QCheckBox * save_aligned_frames_ctl = Q_NULLPTR;
  QBrowsePathCombo * output_aligned_frames_path_ctl = Q_NULLPTR;

  QCheckBox * save_postprocessed_frames_ctl = Q_NULLPTR;
  QBrowsePathCombo * output_postprocessed_frames_path_ctl = Q_NULLPTR;

  QCheckBox * save_accumulation_masks_ctl = Q_NULLPTR;
  QBrowsePathCombo * output_accumulation_masks_path_ctl = Q_NULLPTR;

  QImageProcessorSelectionCombo * accumulated_image_processor_selector_ctl = Q_NULLPTR;

  QCheckBox * write_image_mask_as_alpha_channel_ctl = Q_NULLPTR;
  QCheckBox * dump_reference_data_for_debug_ctl = Q_NULLPTR;


  QToolButton * applyToAll_ctl = Q_NULLPTR;
};

#endif /* __QStackingDebugOptions_h__ */
