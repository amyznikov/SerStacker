/*
 * QLiveImageProcessingOptions.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveImageProcessingOptions_h__
#define __QLiveImageProcessingOptions_h__

#include "QLiveImageProcessingPipeline.h"
#if 0

#include <gui/qgenericpipeline/QGenericImageProcessorOptions.h>

namespace serimager {

class QLiveImageProcessingOptions :
    public QLivePipelineSettings<QLiveImageProcessingPipeline>
{
  Q_OBJECT;
public:
  typedef QLiveImageProcessingOptions ThisClass;
  typedef QLivePipelineSettings<QLiveImageProcessingPipeline> Base;

  QLiveImageProcessingOptions(QWidget * parent = nullptr);
  QLiveImageProcessingOptions(QLiveImageProcessingPipeline * pipeline, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QGenericImageProcessorOptions * genericOptions_ctl = nullptr;
};

} // namespace serimager

#endif // 0

#endif /* __QLiveImageProcessingOptions_h__ */
