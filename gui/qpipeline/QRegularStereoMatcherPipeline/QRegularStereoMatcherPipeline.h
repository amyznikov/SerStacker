/*
 * QRegularStereoMatcherPipeline.h
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRegularStereoMatcherPipeline_h__
#define __QRegularStereoMatcherPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_stereo_matcher_pipeline/c_stereo_matcher_pipeline.h>

class QRegularStereoMatcherPipeline:
    public QImageProcessingPipelineTemplate<c_stereo_matcher_pipeline>
{
public:
  typedef QRegularStereoMatcherPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_stereo_matcher_pipeline> Base;

  QRegularStereoMatcherPipeline(const QString & name, QObject * parent = nullptr) :
    ThisClass(name, nullptr, parent)
  {
  }

  QRegularStereoMatcherPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
    Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }

};

#endif /* __QRegularStereoMatcherPipeline_h__ */
