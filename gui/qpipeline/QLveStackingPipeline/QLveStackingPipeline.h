/*
 * QLveStackingPipeline.h
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLveStackingPipeline_h__
#define __QLveStackingPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_live_stacking_pipeline/c_live_stacking_pipeline.h>

class QLveStackingPipeline :
    public QImageProcessingPipelineTemplate<c_live_stacking_pipeline>
{
public:
  typedef QLveStackingPipeline ThisClass;
  typedef c_live_stacking_pipeline PipelineClass;
  typedef QImageProcessingPipelineTemplate<PipelineClass> Base;

  QLveStackingPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QLveStackingPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const final
  {
    return new QPipelineSettingsWidgetTemplate<PipelineClass>(parent);
  }
};

#endif /* __QLveStackingPipeline_h__ */
