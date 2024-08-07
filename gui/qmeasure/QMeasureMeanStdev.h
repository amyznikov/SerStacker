/*
 * QMeasureMeanStdev.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureMeanStdev_h__
#define __QMeasureMeanStdev_h__

#include "QMeasure.h"

class QMeasureMeanValue:
    public QMeasure
{
public:
  typedef QMeasureMeanValue ThisClass;
  typedef QMeasure Base;

  QMeasureMeanValue();

  QMeasureSettingsWidget* createSettingsWidget(QWidget * parent) const override;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

class QMeanValueMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureMeanValue>
{
public:
  typedef QMeanValueMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureMeanValue> Base;

  QMeanValueMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    averageColorChannels_ctl->setEnabled(false);
    updateControls();
  }
};

class QMeasureStdevValue:
    public QMeasure
{
public:
  typedef QMeasureStdevValue ThisClass;
  typedef QMeasure Base;

  QMeasureStdevValue();

  QMeasureSettingsWidget* createSettingsWidget(QWidget * parent) const override;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};


class QStdevValueMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureStdevValue>
{
public:
  typedef QStdevValueMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureStdevValue> Base;

  QStdevValueMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    averageColorChannels_ctl->setEnabled(false);
    updateControls();
  }
};
#endif /* __QMeasureMeanStdev_h__ */
