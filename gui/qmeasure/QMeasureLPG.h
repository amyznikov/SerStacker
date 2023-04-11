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
  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi, cv::Scalar * value) const override;
  bool hasOptions() const override;
  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;
};

class QLPGMeasureSettingsWidget :
    public QMeasureSettingsWidgetImpl<QMeasureLPG>
{
public:
  typedef QLPGMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetImpl<QMeasureLPG> Base;

  QLPGMeasureSettingsWidget(QWidget * parent = nullptr);

protected:
  QCheckBox * avgc_ctl = nullptr;
  QNumericBox * lw_ctl = nullptr;
  QNumericBox * k_ctl = nullptr;
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * uscale_ctl = nullptr;
  QCheckBox * square_ctl = nullptr;
};

#endif /* __QLPGMeasure_h__ */
