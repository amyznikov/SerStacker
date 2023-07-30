/*
 * QImageStackingPipeline.h
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageStackingPipeline_h__
#define __QImageStackingPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_image_stacking_pipeline/c_image_stacking_pipeline.h>

class QImageStackingPipeline:
    public QImageProcessingPipelineTemplate<c_image_stacking_pipeline>
{
public:
  typedef QImageStackingPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_image_stacking_pipeline> Base;

  QImageStackingPipeline(const QString & name, QObject * parent = nullptr) :
    ThisClass(name, nullptr, parent)
  {
  }

  QImageStackingPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
    Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }
};

#endif /* __QImageStackingPipeline_h__ */
