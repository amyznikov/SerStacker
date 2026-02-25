/*
 * QEpipolarAlignmentPipeline.h
 *
 *  Created on: Dec 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QEpipolarAlignmentPipeline_h__
#define __QEpipolarAlignmentPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_epipolar_alignment_pipeline/c_epipolar_alignment_pipeline.h>

class QEpipolarAlignmentPipeline :
    public QImageProcessingPipelineTemplate<c_epipolar_alignment_pipeline>
{
public:
  typedef QEpipolarAlignmentPipeline ThisClass;
  typedef c_epipolar_alignment_pipeline PipelineClass;
  typedef QImageProcessingPipelineTemplate<PipelineClass> Base;

  QEpipolarAlignmentPipeline();


  QEpipolarAlignmentPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QEpipolarAlignmentPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const final
  {
    return new QPipelineSettingsWidgetTemplate<PipelineClass>(parent);
  }

};

#endif /* __QEpipolarAlignmentPipeline_h__ */
