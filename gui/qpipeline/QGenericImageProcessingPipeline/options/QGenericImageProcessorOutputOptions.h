/*
 * QGenericImageProcessorOutputOptions.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGenericImageProcessorOutputOptions_h__
#define __QGenericImageProcessorOutputOptions_h__

#include <gui/qpipeline/QPipelineOutputOptions.h>
#include <core/pipeline/c_generic_image_processor_pipeline/c_generic_image_processor_pipeline.h>

class QGenericImageProcessorOutputOptions :
    public QPipelineOutputOptions<c_generic_image_processor_output_options>
{
public:
  typedef QGenericImageProcessorOutputOptions ThisClass;
  typedef QPipelineOutputOptions<c_generic_image_processor_output_options> Base;

  QGenericImageProcessorOutputOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  QCheckBox * save_processed_frames_ctl = nullptr;
  QLineEditBox * processed_frames_filename_ctl = nullptr;
};

#endif /* __QGenericImageProcessorOutputOptions_h__ */
