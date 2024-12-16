/*
 * QCTEPipeline.h
 *
 *  Created on: Dec 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCTEPipeline_h__
#define __QCTEPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_cte_pipeline/c_cte_pipeline.h>

class QCTEPipeline :
    public QImageProcessingPipelineTemplate<c_cte_pipeline>
{
public:
  typedef QCTEPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_cte_pipeline> Base;

  QCTEPipeline();


  QCTEPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QCTEPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }

};

#endif /* __QCTEPipeline_h__ */
