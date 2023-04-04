/*
 * QGenericImageProcessorOptions.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGenericImageProcessorOptions_h__
#define __QGenericImageProcessorOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/pipeline/generic/c_generic_image_processor.h>
#include "QGenericImageProcessorOutputOptions.h"

class QGenericImageProcessorOptions :
    public QSettingsWidget
{
public:
  typedef QGenericImageProcessorOptions ThisClass;
  typedef QSettingsWidget Base;

  QGenericImageProcessorOptions(QWidget * parent = nullptr);

  void set_pipeline(c_generic_image_processor * pipeline);
  c_generic_image_processor * pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_generic_image_processor * pipeline_ = nullptr;

  QImageProcessorSelectionCombo * frame_processor_ctl = nullptr;
  QGenericImageProcessorOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QGenericImageProcessorOptions_h__ */
