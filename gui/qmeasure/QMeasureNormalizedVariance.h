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
  bool hasOptions() const override;
  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
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
