/*
 * QMeasureLPG.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLPGMeasure_h__
#define __QLPGMeasure_h__

#include "QMeasure.h"
#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>

class QMeasureLPG:
    public QMeasure,
    public c_lpg_sharpness_measure
{
public :
  typedef QMeasureLPG ThisClass;
  typedef QMeasure Base;

  QMeasureLPG();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QLPGMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureLPG>
{
public:
  typedef QLPGMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureLPG> Base;

  QLPGMeasureSettingsWidget(QWidget * parent = nullptr);

protected:
  QNumericBox * lw_ctl = nullptr;
  QNumericBox * k_ctl = nullptr;
  QNumericBox * p_ctl = nullptr;
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * uscale_ctl = nullptr;
};

#endif /* __QLPGMeasure_h__ */
