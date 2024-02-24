/*
 * QRunningAveragePipeline.h
 *
 *  Created on: Feb 24, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRunningAveragePipeline_h__
#define __QRunningAveragePipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_running_average_pipeline/c_running_average_pipeline.h>

class QRunningAveragePipeline :
    public QImageProcessingPipelineTemplate<c_running_average_pipeline>
{
public:
  typedef QRunningAveragePipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_running_average_pipeline> Base;

  QRunningAveragePipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QRunningAveragePipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }
};


#endif /* __QRunningAveragePipeline_h__ */
