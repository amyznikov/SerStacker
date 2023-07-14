/*
 * QStackOutputOptions.h
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStackingDebugOptions_h__
#define __QStackingDebugOptions_h__

#include <core/pipeline/c_image_stacking_pipeline/c_image_stacking_pipeline.h>
#include "../../QPipelineOutputOptions.h"

class QStackOutputOptions :
    public QPipelineOutputOptions<c_image_stacking_output_options>
{
  Q_OBJECT;
public:
  typedef QStackOutputOptions ThisClass;
  typedef QPipelineOutputOptions<c_image_stacking_output_options> Base;

  QStackOutputOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  QCheckBox * save_preprocessed_frames_ctl = nullptr;
  QLineEditBox * output_preprocessed_frames_path_ctl = nullptr;

  QCheckBox * save_aligned_frames_ctl = nullptr;
  QLineEditBox * output_aligned_frames_path_ctl = nullptr;

  QCheckBox * save_ecc_frames_ctl = nullptr;
  QLineEditBox * output_ecc_frames_path_ctl = nullptr;

  QCheckBox * save_postprocessed_frames_ctl = nullptr;
  QLineEditBox * output_processed_aligned_frames_path_ctl = nullptr;

  QCheckBox * save_accumulated_frames_ctl = nullptr;
  QLineEditBox * output_accumulated_frames_path_ctl = nullptr;

  QCheckBox * save_accumulation_masks_ctl = nullptr;
  QLineEditBox * output_accumulation_masks_path_ctl = nullptr;

  QCheckBox * write_image_mask_as_alpha_channel_ctl = nullptr;
  QCheckBox * dump_reference_data_for_debug_ctl = nullptr;

  QCheckBox * debug_frame_registration_ctl = nullptr;
  QLineEditBox * debug_frame_registration_frame_indexes_ctl = nullptr;


//  QToolButton * applyToAll_ctl = nullptr;
};

#endif /* __QStackingDebugOptions_h__ */
