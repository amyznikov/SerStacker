/*
 * QRegularStereoMatcherPipeline.h
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRegularStereoMatcherPipeline_h__
#define __QRegularStereoMatcherPipeline_h__

#include <core/pipeline/c_stereo_matcher_pipeline/c_stereo_matcher_pipeline.h>
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <gui/qpipeline/stereo/QStereoRectificationOptions.h>
#include <gui/qpipeline/stereo/QStereoMatcherOptions.h>
#include "options/QStereoMatcherInputOptions.h"
#include "options/QStereoMatcherProcessingOptions.h"
#include "options/QStereoMatcherOutputOptions.h"

class QRegularStereoMatcherPipeline;
class QRegularStereoMatcherSettingsWidget;


class QRegularStereoMatcherSettingsWidget :
    public QPipelineSettingsWidgetBase<QRegularStereoMatcherPipeline>
{
public:
  typedef QRegularStereoMatcherSettingsWidget  ThisClass;
  typedef QPipelineSettingsWidgetBase<QRegularStereoMatcherPipeline> Base;

  QRegularStereoMatcherSettingsWidget(QWidget * parent = nullptr);
  QRegularStereoMatcherSettingsWidget(const QString & prefix, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QStereoMatcherInputOptions * inputOptions_ctl = nullptr;
  QStereoRectificationOptions * stereoRectificationOptions_ctl = nullptr;
  QStereoMatcherProcessingOptions * processingOptions_ctl = nullptr;
  QStereoMatcherOptions * stereoMatcherOptions_ctl = nullptr;
  QStereoMatcherOutputOptions * outputOptions_ctl = nullptr;
};

class QRegularStereoMatcherPipeline:
    public QImageProcessingPipelineBase<c_stereo_matcher_pipeline>
{
public:
  typedef QRegularStereoMatcherPipeline ThisClass;
  typedef QImageProcessingPipelineBase<c_stereo_matcher_pipeline> Base;

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
    return new QRegularStereoMatcherSettingsWidget(parent);
  }

};

#endif /* __QRegularStereoMatcherPipeline_h__ */
