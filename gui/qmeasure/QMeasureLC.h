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

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

//  void setAverageColorChannels(bool v) override;
//  bool averageColorChannels() const override;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QLCMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureLC>
{
public:
  typedef QLCMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureLC> Base;

  QLCMeasureSettingsWidget(QWidget * parent = nullptr);

protected:
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
};

#endif /* __QMeasureLC_h__ */
