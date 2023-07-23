/*
 * QGenericImageProcessingPipeline.h
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGenericImageProcessingPipeline_h__
#define __QGenericImageProcessingPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <gui/qpipeline/QPipelineSettingsCtrl.h>
//#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/pipeline/c_generic_image_processor_pipeline/c_generic_image_processor_pipeline.h>
//#include "options/QGenericImageProcessorInputOptions.h"
//#include "options/QGenericImageProcessorOutputOptions.h"


class QGenericImageProcessingPipeline;
class QGenericImageProcessingSettingsWidget;

class QGenericImageProcessingSettingsWidget :
    public QPipelineSettingsWidgetBase<QGenericImageProcessingPipeline>
{
public:
  typedef QGenericImageProcessingSettingsWidget  ThisClass;
  typedef QPipelineSettingsWidgetBase<QGenericImageProcessingPipeline> Base;

  QGenericImageProcessingSettingsWidget(QWidget * parent = nullptr);
  QGenericImageProcessingSettingsWidget(const QString & prefix, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QPipelineSettingsCtrl * settings_ctl = nullptr;
};

class QGenericImageProcessingPipeline :
    public QImageProcessingPipelineBase<c_generic_image_processor_pipeline>
{
public:
  typedef QGenericImageProcessingPipeline ThisClass;
  typedef QImageProcessingPipelineBase<c_generic_image_processor_pipeline> Base;

  QGenericImageProcessingPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QGenericImageProcessingPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QGenericImageProcessingSettingsWidget(parent);
  }
};

#endif /* __QGenericImageProcessingPipeline_h__ */
