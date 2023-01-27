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
//#include <gui/qimproc/QImageProcessorsCollection.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QStackOutputOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QStackOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QStackOutputOptions(QWidget * parent = nullptr);

  void set_stacking_options(const c_image_stacking_options::ptr & options);
  const c_image_stacking_options::ptr & stacking_options() const;

Q_SIGNALS:
  void applyOutputOptionsToAllRequested(const c_image_stacking_output_options & options);


protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_options::ptr options_ = nullptr;

  QBrowsePathCombo * output_directory_ctl = nullptr;

  QCheckBox * save_preprocessed_frames_ctl = nullptr;
  QBrowsePathCombo * output_preprocessed_frames_path_ctl = nullptr;

  QCheckBox * save_aligned_frames_ctl = nullptr;
  QBrowsePathCombo * output_aligned_frames_path_ctl = nullptr;

  QCheckBox * save_ecc_frames_ctl = nullptr;
  QBrowsePathCombo * output_ecc_frames_path_ctl = nullptr;

  QCheckBox * save_postprocessed_frames_ctl = nullptr;
  QBrowsePathCombo * output_processed_aligned_frames_path_ctl = nullptr;

  QCheckBox * save_accumulated_frames_ctl = nullptr;
  QBrowsePathCombo * output_accumulated_frames_path_ctl = nullptr;

  QCheckBox * save_accumulation_masks_ctl = nullptr;
  QBrowsePathCombo * output_accumulation_masks_path_ctl = nullptr;

  QCheckBox * write_image_mask_as_alpha_channel_ctl = nullptr;
  QCheckBox * dump_reference_data_for_debug_ctl = nullptr;

  QCheckBox * debug_frame_registration_ctl = nullptr;
  QLineEditBox * debug_frame_registration_frame_indexes_ctl = nullptr;


//  QToolButton * applyToAll_ctl = nullptr;
};

#endif /* __QStackingDebugOptions_h__ */
