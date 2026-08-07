/*
 * QFrameQualityEstimatorSettingsWidget.h
 *
 *  Created on: Aug 7, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFrameQualityEstimatorSettingsWidget_h__
#define __QFrameQualityEstimatorSettingsWidget_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QFrameQualityEstimation.h"

namespace serimager {

class QFrameQualityEstimatorSettingsWidget :
    public QSettingsWidgetTemplate<QFrameQualityEstimation>
{
public:
  typedef QFrameQualityEstimatorSettingsWidget ThisClass;
  typedef QSettingsWidgetTemplate<QFrameQualityEstimation> Base;

  QFrameQualityEstimatorSettingsWidget(QWidget * parent = nullptr);

  void setFrameQualityEstimator(QFrameQualityEstimation * estimator);
  QFrameQualityEstimation * frameQualityEstimator() const;

protected:
  QCheckBox * enabled_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QFrameQualityEstimatorSettingsWidget_h__ */
