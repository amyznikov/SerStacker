/*
 * QMeasureSharpnessNorm.h
 *
 *  Created on: Apr 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureSharpnessNorm_h__
#define __QMeasureSharpnessNorm_h__

#include "QMeasure.h"
#include <core/proc/sharpness_measure/c_sharpness_norm_measure.h>

class QMeasureSharpnessNorm:
    public QMeasure,
    public c_sharpness_norm_measure
{
public :
  typedef QMeasureSharpnessNorm ThisClass;
  typedef QMeasure Base;

  QMeasureSharpnessNorm();
  bool hasOptions() const override;
  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

class QSharpnessNormMeasureSettingsWidget :
    public QMeasureSettingsWidgetImpl<QMeasureSharpnessNorm>
{
public:
  typedef QSharpnessNormMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetImpl<QMeasureSharpnessNorm> Base;

  QSharpnessNormMeasureSettingsWidget(QWidget * parent = nullptr);

protected:
  using NormType = c_sharpness_norm_measure::NormType;
  QEnumComboBox<NormType> * norm_type_ctl = nullptr;
  QNumericBox * sigma_ctl = nullptr;
};

#endif /* __QMeasureSharpnessNorm_h__ */
