/*
 * QMeasureNoise.h
 *
 *  Created on: Apr 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureNoise_h__
#define __QMeasureNoise_h__

#include "QMeasure.h"

class QMeasureNoise:
    public QMeasure
{
public:
  typedef QMeasureNoise ThisClass;
  typedef QMeasure Base;

  QMeasureNoise();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QNoiseMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureNoise>
{
public:
  typedef QNoiseMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureNoise> Base;

  QNoiseMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
  }

};

#endif /* __QMeasureNoise_h__ */
