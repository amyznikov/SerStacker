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
#include <core/pipeline/c_regular_stereo_pipeline.h>

class QRStereoImageProcessingOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoImageProcessingOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoImageProcessingOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline);
  const c_regular_stereo_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;
  void populatesources();

protected:
  c_regular_stereo_pipeline::sptr pipeline_;

  QImageProcessorSelectionCombo * input_image_processor_ctl = nullptr;
  QImageProcessorSelectionCombo * stereo_match_preprocessor_ctl = nullptr;
  QImageProcessorSelectionCombo * output_image_processor_ctl = nullptr;
};

#endif /* __QRStereoImageProcessingOptions_h__ */
