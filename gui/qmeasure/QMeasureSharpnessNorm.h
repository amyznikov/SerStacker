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
  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QSharpnessNormMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureSharpnessNorm>
{
public:
  typedef QSharpnessNormMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureSharpnessNorm> Base;

  QSharpnessNormMeasureSettingsWidget(QWidget * parent = nullptr);

protected:
  QEnumComboBox<cv::NormTypes> * norm_type_ctl = nullptr;
  QNumericBox * sigma_ctl = nullptr;
};

#endif /* __QMeasureSharpnessNorm_h__ */
