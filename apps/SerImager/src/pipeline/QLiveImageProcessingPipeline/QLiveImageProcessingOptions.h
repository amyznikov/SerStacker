/*
 * QLiveImageProcessingOptions.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveImageProcessingOptions_h__
#define __QLiveImageProcessingOptions_h__

#include "QLiveImageProcessingPipeline.h"
#include <gui/qgenericpipeline/QGenericImageProcessorOptions.h>

namespace serimager {

class QLiveImageProcessingOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLiveImageProcessingOptions ThisClass;
  typedef QSettingsWidget Base;

  QLiveImageProcessingOptions(QWidget * parent = nullptr);
  QLiveImageProcessingOptions(QLiveImageProcessingPipeline * pipeline_, QWidget * parent = nullptr);

  void setPipeline(QLiveImageProcessingPipeline * pipeline);
  QLiveImageProcessingPipeline * pipeline() const;

//protected Q_SLOTS:
//  void onLivePipelineStateChanged(bool isRunning);

protected:
  void onupdatecontrols() override;

protected:
  QLiveImageProcessingPipeline * pipeline_ = nullptr;
  QGenericImageProcessorOptions * genericOptions_ctl = nullptr;
};

} // namespace serimager

#endif /* __QLiveImageProcessingOptions_h__ */
