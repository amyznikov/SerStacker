/*
 * QMeasureLC.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureLC_h__
#define __QMeasureLC_h__

#include "QMeasure.h"
#include <core/proc/sharpness_measure/c_local_contrast_measure.h>

class QMeasureLC:
    public QMeasure,
    public c_local_contrast_measure
{
public :
  typedef QMeasureLC ThisClass;
  typedef QMeasure Base;

  QMeasureLC();
  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi, cv::Scalar * value) const override;
  bool hasOptions() const override;
  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

};

class QLCMeasureSettingsWidget :
    public QMeasureSettingsWidgetImpl<QMeasureLC>
{
public:
  typedef QLCMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetImpl<QMeasureLC> Base;

  QLCMeasureSettingsWidget(QWidget * parent = nullptr);

protected:
  QCheckBox * avgc_ctl = nullptr;
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
};

#endif /* __QMeasureLC_h__ */
