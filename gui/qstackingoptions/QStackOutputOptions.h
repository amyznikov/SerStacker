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

  void set_output_options(c_image_stacking_output_options * options);
  const c_image_stacking_output_options * output_options() const;

signals:
  void applyOutputOptionsToAllRequested(const c_image_stacking_output_options & options);


protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_output_options * options_ = Q_NULLPTR;

  QBrowsePathCombo * output_directory_ctl = Q_NULLPTR;
  QCheckBox * write_image_mask_as_alpha_channel_ctl = Q_NULLPTR;
  QCheckBox * write_aligned_video_ctl = Q_NULLPTR;
  QBrowsePathCombo * output_aligned_video_filename_ctl = Q_NULLPTR;
  QCheckBox * dump_reference_data_for_debug_ctl = Q_NULLPTR;

  QCheckBox * save_processed_frames_ctl = Q_NULLPTR;

  QImageProcessorSelectionCombo * frame_processor_selector_ctl = Q_NULLPTR;
  QImageProcessorSelectionCombo * accumulated_image_processor_selector_ctl = Q_NULLPTR;


  QToolButton * applyToAll_ctl = Q_NULLPTR;
};

#endif /* __QStackingDebugOptions_h__ */
