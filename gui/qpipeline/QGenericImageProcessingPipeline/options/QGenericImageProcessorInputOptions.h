/*
 * QGenericImageProcessorInputOptions.h
 *
 *  Created on: Jul 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGenericImageProcessorInputOptions_h__
#define __QGenericImageProcessorInputOptions_h__

#include <gui/qpipeline/QPipelineInputOptions.h>
#include <core/pipeline/c_generic_image_processor_pipeline/c_generic_image_processor_pipeline.h>

class QGenericImageProcessorInputOptions :
    public QPipelineInputOptions<c_generic_image_processor_pipeline>
{
public:
  typedef QGenericImageProcessorInputOptions ThisClass;
  typedef QPipelineInputOptions<c_generic_image_processor_pipeline> Base;

  QGenericImageProcessorInputOptions(QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;
};

#endif /* __QGenericImageProcessorInputOptions_h__ */
