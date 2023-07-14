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
#include "options/QFrameAccumulationOptions.h"
#include "options/QFrameRegistrationOptions.h"
#include "options/QFrameUpscaleOptions.h"
#include "options/QImageProcessingOptions.h"
#include "options/QImageStackingInputOptions.h"
#include "options/QROISelectionOptions.h"
#include "options/QStackOutputOptions.h"

class QImageStackingPipeline;
class QImageStackingSettingsWidget;

class QImageStackingSettingsWidget :
    public QPipelineSettingsWidgetBase<QImageStackingPipeline>
{
public:
  typedef QImageStackingSettingsWidget ThisClass;
  typedef QPipelineSettingsWidgetBase<QImageStackingPipeline> Base;

  QImageStackingSettingsWidget(QWidget * parent = nullptr);
  QImageStackingSettingsWidget(const QString & prefix, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QImageStackingInputOptions * inputOptions_ctl = nullptr;
  QROISelectionOptions * roiSelection_ctl = nullptr;
  QFrameUpscaleOptions * upscaleOptions_ctl = nullptr;
  QFrameRegistrationOptions * frameRegistration_ctl = nullptr;
  QFrameAccumulationOptions * frameAccumulation_ctl = nullptr;
  QImageProcessingOptions * imageProcessingOptions_ctl = nullptr;
  QStackOutputOptions * outputOptions_ctl = nullptr;
};

class QImageStackingPipeline:
    public QImageProcessingPipelineBase<c_image_stacking_pipeline>
{
public:
  typedef QImageStackingPipeline ThisClass;
  typedef QImageProcessingPipelineBase<c_image_stacking_pipeline> Base;

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
    return new QImageStackingSettingsWidget(parent);
  }
};

#endif /* __QImageStackingPipeline_h__ */
