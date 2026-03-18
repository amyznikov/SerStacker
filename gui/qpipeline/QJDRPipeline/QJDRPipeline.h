/*
 * QJDRPipeline.h
 *
 *  Created on: Mar 17, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QJDRPipeline_h__
#define __QJDRPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_jdr_pipeline/c_jdr_pipeline.h>

class QJDRPipeline :
    public QImageProcessingPipelineTemplate<c_jdr_pipeline>
{
public:
  typedef QJDRPipeline ThisClass;
  typedef c_jdr_pipeline PipelineClass;
  typedef QImageProcessingPipelineTemplate<PipelineClass> Base;

  QJDRPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QJDRPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const final
  {
    return new QPipelineSettingsWidgetTemplate<PipelineClass>(parent);
  }

};

#endif /* __QJDRPipeline_h__ */
