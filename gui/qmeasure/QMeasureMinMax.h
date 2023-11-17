/*
 * QMeasureMinMax.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureMinMax_h__
#define __QMeasureMinMax_h__

#include "QMeasure.h"

class QMeasureMinValue :
    public QMeasure
{
public:
  typedef QMeasureMinValue ThisClass;
  typedef QMeasure Base;

  QMeasureMinValue();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

class QMinValueMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureMinValue>
{
public:
  typedef QMinValueMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureMinValue> Base;

  QMinValueMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    updateControls();
  }
};


class QMeasureMaxValue :
    public QMeasure
{
public:
  typedef QMeasureMaxValue ThisClass;
  typedef QMeasure Base;

  QMeasureMaxValue();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

class QMaxValueMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureMaxValue>
{
public:
  typedef QMaxValueMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureMaxValue> Base;

  QMaxValueMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    updateControls();
  }
};



#endif /* __QMeasureMinMax_h__ */
