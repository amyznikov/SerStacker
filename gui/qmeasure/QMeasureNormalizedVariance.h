/*
 * QMeasureNormalizedVariance.h
 *
 *  Created on: Apr 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureNormalizedVariance_h__
#define __QMeasureNormalizedVariance_h__

#include "QMeasure.h"
#include <core/proc/sharpness_measure/c_normalized_variance_measure.h>

class QMeasureNormalizedVariance:
    public QMeasure,
    public c_normalized_variance_measure
{
public:
  typedef QMeasureNormalizedVariance ThisClass;
  typedef QMeasure Base;

  QMeasureNormalizedVariance();
  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi, cv::Scalar * value) const override;
  bool hasOptions() const override;
  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;
};

class QNormalizedVarianceSettingsWidget :
    public QMeasureSettingsWidgetImpl<QMeasureNormalizedVariance>
{
public:
  typedef QNormalizedVarianceSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetImpl<QMeasureNormalizedVariance> Base;

  QNormalizedVarianceSettingsWidget(QWidget * parent = nullptr);

protected:
  QCheckBox * avgc_ctl = nullptr;
};

#endif /* __QMeasureNormalizedVariance_h__ */
