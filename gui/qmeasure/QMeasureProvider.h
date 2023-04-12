/*
 * QMeasureProvider.h
 *
 *  Created on: Apr 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureProvider_h__
#define __QMeasureProvider_h__

#include "QMeasure.h"

class QMeasureProvider :
    public QObject
{
  Q_OBJECT;
public:
  typedef QMeasureProvider ThisClass;
  typedef QObject Base;

  struct MeasuredValue
  {
    const QMeasure * measure;
    const cv::Scalar value;
    const int cn = 0;

    MeasuredValue(const QMeasure * _measure, const cv::Scalar & _value, int _cn) :
        measure(_measure),
        value(_value),
        cn(_cn)
    {
    }
  };

  struct MeasuredFrame
  {
    std::vector<MeasuredValue> measurements;

    template<typename ... _Args>
    MeasuredValue& emplace_back(_Args && ... __args)
    {
      measurements.emplace_back(std::forward<_Args>(__args)...);
      return measurements.back();
    }

    bool empty() const
    {
      return measurements.empty();
    }
  };

  QMeasureProvider(QObject * parent = nullptr);

  const std::set<QMeasure*> measures() const;


  bool compute(cv::InputArray image, cv::InputArray mask, const QRect & roi);
  bool compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi);

  const std::deque<MeasuredFrame> & measured_frames() const;
  void clear_measured_frames();

  void set_max_measured_frames(int v);
  int max_measured_frames() const;

Q_SIGNALS:
  void measurementsChanged();

protected:
  std::deque<MeasuredFrame> measured_frames_;
  std::set<QMeasure*> measures_;
  int max_measured_frames_ = 200;

  //static QMeasure * available_measures_[];
};

#endif /* __QMeasureProvider_h__ */
