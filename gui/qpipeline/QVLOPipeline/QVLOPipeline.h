/*
 * QVLOPipeline.h
 *
 *  Created on: Oct 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QVLOPipeline_h__
#define __QVLOPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_vlo_pipeline/c_vlo_pipeline.h>

class QVLOPipeline:
    public QImageProcessingPipelineTemplate<c_vlo_pipeline>
{
public:
  typedef QVLOPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_vlo_pipeline> Base;

  QVLOPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QVLOPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }

};

#endif /* __QVLOPipeline_h__ */
