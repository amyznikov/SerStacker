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

///////////////////////////////////////////////////////////////////////////////////////

class QMeasureMeanValue: public QMeasure
{
public:
  typedef QMeasureMeanValue ThisClass;
  typedef QMeasure Base;

  QMeasureMeanValue();

  QMeasureSettingsWidget* createSettingsWidget(QWidget * parent) const final;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QMeanValueMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureMeanValue>
{
public:
  typedef QMeanValueMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureType> Base;

  QMeanValueMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    updateControls();
  }
};

///////////////////////////////////////////////////////////////////////////////////////

class QMeasureStdevValue: public QMeasure
{
public:
  typedef QMeasureStdevValue ThisClass;
  typedef QMeasure Base;

  QMeasureStdevValue();

  QMeasureSettingsWidget* createSettingsWidget(QWidget * parent) const final;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};


class QStdevValueMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureStdevValue>
{
public:
  typedef QStdevValueMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureType> Base;

  QStdevValueMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    updateControls();
  }
};

///////////////////////////////////////////////////////////////////////////////////////
class QMeasureSum: public QMeasure
{
public:
  typedef QMeasureSum ThisClass;
  typedef QMeasure Base;

  QMeasureSum();

  QMeasureSettingsWidget* createSettingsWidget(QWidget * parent) const final;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QMeasureSumSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureMeanValue>
{
public:
  typedef QMeasureSumSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureType> Base;

  QMeasureSumSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    updateControls();
  }
};

#endif /* __QMeasureMeanStdev_h__ */
