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
#include "options/QLveStackingInputOptions.h"
#include "options/QLiveStackingAccumulateOptions.h"
#include "options/QLveStackingOutputOptions.h"

class QLveStackingPipeline;
class QLveStackingSettingsWidget;

class QLveStackingSettingsWidget :
    public QPipelineSettingsWidgetBase<QLveStackingPipeline>
{
public:
  typedef QLveStackingSettingsWidget ThisClass;
  typedef QPipelineSettingsWidgetBase<QLveStackingPipeline> Base;

  QLveStackingSettingsWidget(QWidget * parent = nullptr);
  QLveStackingSettingsWidget(const QString & prefix, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QLveStackingInputOptions * inputOptions_ctl = nullptr;
  QLiveStackingAccumulateOptions * accumulate_ctl = nullptr;
  QLveStackingOutputOptions * outputOptions_ctl = nullptr;
};

class QLveStackingPipeline :
    public QImageProcessingPipelineBase<c_live_stacking_pipeline>
{
public:
  typedef QLveStackingPipeline ThisClass;
  typedef QImageProcessingPipelineBase<c_live_stacking_pipeline> Base;

  QLveStackingPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QLveStackingPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QLveStackingSettingsWidget(parent);
  }
};

#endif /* __QLveStackingPipeline_h__ */
