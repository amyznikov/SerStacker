/*
 * QMeasureBaryCenter.h
 *
 *  Created on: May 30, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureBaryCenter_h__
#define __QMeasureBaryCenter_h__

#include "QMeasure.h"

class QMeasureBaryCenter :
    public QMeasure
{
public:
  typedef QMeasureBaryCenter ThisClass;
  typedef QMeasure Base;
  QMeasureBaryCenter(const QString & name, const QString & tooltip);
  QMeasureSettingsWidget* createSettingsWidget(QWidget * parent) const final;
  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Point2d pos[4]) const;
};

class QMeasureBaryCenterX :
    public QMeasureBaryCenter
{
public:
  typedef QMeasureBaryCenterX ThisClass;
  typedef QMeasureBaryCenter Base;
  QMeasureBaryCenterX();
  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QMeasureBaryCenterY :
    public QMeasureBaryCenter
{
public:
  typedef QMeasureBaryCenterY ThisClass;
  typedef QMeasureBaryCenter Base;
  QMeasureBaryCenterY();
  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QMeasureBaryCenterSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureBaryCenter>
{
public:
  typedef QMeasureBaryCenterSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureType> Base;

  QMeasureBaryCenterSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    updateControls();
  }
};

#endif /* __QMeasureBaryCenter_h__ */
