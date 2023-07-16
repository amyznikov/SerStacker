/*
 * QLveStackingInputOptions.h
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLveStackingInputOptions_h__
#define __QLveStackingInputOptions_h__

#include <gui/qpipeline/QPipelineInputOptions.h>
#include <core/pipeline/c_live_stacking_pipeline/c_live_stacking_pipeline.h>


class QLveStackingInputOptions :
    public QPipelineInputOptions<c_live_stacking_pipeline>
{
public:
  typedef QLveStackingInputOptions ThisClass;
  typedef QPipelineInputOptions<c_live_stacking_pipeline> Base;

  QLveStackingInputOptions(QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;
};


#endif /* __QLveStackingInputOptions_h__ */
