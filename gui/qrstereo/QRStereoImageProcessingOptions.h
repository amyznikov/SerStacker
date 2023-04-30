/*
 * QRStereoImageProcessingOptions.h
 *
 *  Created on: Mar 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoImageProcessingOptions_h__
#define __QRStereoImageProcessingOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/pipeline/rstereo/c_regular_stereo.h>

class QRStereoImageProcessingOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoImageProcessingOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoImageProcessingOptions(QWidget * parent = nullptr);

  void set_options(c_regular_stereo_image_processing_options * options);
  c_regular_stereo_image_processing_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_regular_stereo_image_processing_options * options_ = nullptr;

  QImageProcessorSelectionCombo * input_image_processor_ctl = nullptr;
  QImageProcessorSelectionCombo * remapped_image_processor_ctl = nullptr;
  QImageProcessorSelectionCombo * output_image_processor_ctl = nullptr;
};

#endif /* __QRStereoImageProcessingOptions_h__ */
