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

  static QMeasureProvider * instance();

  static const std::vector<QMeasure*> & measures();

  static bool compute(cv::InputArray image, cv::InputArray mask, const QRect & roi);
  static bool compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi);

  static const std::set<const std::set<QMeasure*>*>& requested_measures();
  static void request_measures(const std::set<QMeasure*> * r);
  static void remove_measure_request(const std::set<QMeasure*> * r);

  static const std::deque<MeasuredFrame> & measured_frames();
  static void clear_measured_frames();

  static void set_max_measured_frames(int v);
  static int max_measured_frames();

Q_SIGNALS:
  void measurementsChanged();

protected:
  QMeasureProvider(QObject * parent = nullptr);

protected:
  static std::set<const std::set<QMeasure*>*> requested_measures_;
  static std::deque<MeasuredFrame> measured_frames_;
  static std::vector<QMeasure*> measures_;
  static int max_measured_frames_;
};

#endif /* __QMeasureProvider_h__ */
