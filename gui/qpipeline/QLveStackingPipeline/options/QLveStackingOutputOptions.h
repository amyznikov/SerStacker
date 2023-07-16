/*
 * QLveStackingOutputOptions.h
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLveStackingOutputOptions_h__
#define __QLveStackingOutputOptions_h__

#include <gui/qpipeline/QPipelineOutputOptions.h>
#include <core/pipeline/c_live_stacking_pipeline/c_live_stacking_pipeline.h>

class QLveStackingOutputOptions :
    public QPipelineOutputOptions<c_live_stacking_output_options>
{
public:
  typedef QLveStackingOutputOptions ThisClass;
  typedef QPipelineOutputOptions<c_live_stacking_output_options> Base;

  QLveStackingOutputOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  //  QCheckBox * save_processed_frames_ctl = nullptr;
  //  QLineEditBox * processed_frames_filename_ctl = nullptr;
};


#endif /* __QLveStackingOutputOptions_h__ */
