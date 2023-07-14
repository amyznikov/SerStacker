/*
 * QImageProcessingOptions.h
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageProcessingOptions_h__
#define __QImageProcessingOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/pipeline/c_image_stacking_pipeline/c_image_stacking_pipeline.h>

class QImageProcessingOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageProcessingOptions ThisClass;
  typedef QSettingsWidget Base;

  QImageProcessingOptions(QWidget * parent = nullptr);

  void set_image_processing_options(c_image_processing_options * options);
  c_image_processing_options * image_processing_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_image_processing_options * options_ = nullptr;
  QImageProcessorSelectionCombo * input_image_processor_ctl = nullptr;
  QImageProcessorSelectionCombo * ecc_image_processor_ctl = nullptr;
  QImageProcessorSelectionCombo * aligned_image_processor_ctl = nullptr;
  QImageProcessorSelectionCombo * incremental_frame_processor_ctl = nullptr;
  QImageProcessorSelectionCombo * accumulated_image_processor_ctl = nullptr;
};


#endif /* __QImageProcessingOptions_h__ */
