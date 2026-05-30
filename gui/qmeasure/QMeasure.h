/*
 * QMeasure.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasure_h__
#define __QMeasure_h__

#include <gui/widgets/QSettingsWidget.h>
#include <opencv2/opencv.hpp>

class QMeasureSettingsWidget;

class QMeasure
{
public:
  QMeasure(const QString & name, const QString & tooltip) :
      _name(name),
      _tooltip(tooltip)
  {
  }

  virtual ~QMeasure() = default;

  const QString & name() const
  {
    return _name;
  }

  const QString & tooltip() const
  {
    return _tooltip;
  }

  virtual QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const = 0;

  virtual int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const = 0;

protected:

protected:
  const QString _name;
  const QString _tooltip;
};

Q_DECLARE_METATYPE(const QMeasure*);
Q_DECLARE_METATYPE(QMeasure*);


class QMeasureSettingsWidget:
    public QSettingsWidget
{
public:
  typedef QMeasureSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
  }

  virtual void setCurrentMeasure(QMeasure * m) = 0;
  virtual QMeasure * currentMeasure() const = 0;
};

template<class MeasureType>
class QMeasureSettingsWidgetTemplate:
    public QMeasureSettingsWidget
{
public:
  typedef QMeasureSettingsWidgetTemplate ThisClass;
  typedef QMeasureSettingsWidget Base;
  typedef MeasureType QMeasureType;

  QMeasureSettingsWidgetTemplate(QWidget * parent = nullptr) :
      Base(parent)
  {
  }

  void setMeasure(MeasureType * m)
  {
    _measure = m;
    updateControls();
  }

  MeasureType* measure() const
  {
    return _measure;
  }

  void setCurrentMeasure(QMeasure * m) override
  {
    setMeasure(dynamic_cast<MeasureType*>(m));
  }

  QMeasure* currentMeasure() const override
  {
    return _measure;
  }

protected:

  // placeholder for overrides
  virtual void update_measure_controls()
  {
    setEnabled(true);
  }

protected:
  MeasureType *_measure = nullptr;
};

class QMeasureCentralPixelValue:
    public QMeasure
{
public:
  typedef QMeasureCentralPixelValue ThisClass;
  typedef QMeasure Base;

  QMeasureCentralPixelValue();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const final;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};


class QMeasureCentralPixelValueSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureCentralPixelValue>
{
public:
  typedef QMeasureCentralPixelValueSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureCentralPixelValue> Base;

  QMeasureCentralPixelValueSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    updateControls();
  }
};



#endif /* __QMeasure_h__ */
