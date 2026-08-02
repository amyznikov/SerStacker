/*
 * QMeasureLocalVariance.h
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureLocalVariance_h__
#define __QMeasureLocalVariance_h__

#include "QMeasure.h"
#include <core/proc/sharpness_measure/c_local_variance_sharpness_measure.h>

class QMeasureLocalVariance : public QMeasure,
    public c_local_variance_sharpness_measure
{
public :
  typedef QMeasureLocalVariance ThisClass;
  typedef QMeasure Base;

  QMeasureLocalVariance();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const final;
  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QLocalVarianceMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureLocalVariance>
{
public:
  typedef QLocalVarianceMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureLocalVariance> Base;

  QLocalVarianceMeasureSettingsWidget(QWidget * parent = nullptr);

protected:
  QNumericBox * p_ctl = nullptr;
  QNumericBox * kradius_ctl = nullptr;
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * uscale_ctl = nullptr;
};

#endif /* __QMeasureLocalVariance_h__ */
