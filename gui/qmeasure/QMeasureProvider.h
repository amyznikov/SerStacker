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
  typedef std::set<QMeasure*,std::less<QMeasure*>> MeasuresCollection;
  typedef std::set<const MeasuresCollection*,std::less<const MeasuresCollection*>> MeasureRequest;

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
    QString dataChannel;
    std::vector<MeasuredValue> measurements;
    cv::Scalar mcc;
    int mcn = 0;

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

  static const std::vector<QMeasure*> & available_measures();
  static const QMeasure* valueMeasure();

  static bool compute(MeasuredFrame * frame, cv::InputArray image, cv::InputArray mask, const QRect & roi);
  static bool compute(MeasuredFrame * frame, cv::InputArray image, cv::InputArray mask, const cv::Rect & roi);

  static const QStringList & requested_channels();
  static void set_requested_channels(QStringList & v);
  static const MeasureRequest& requested_measures();
  static void request_measures(const MeasuresCollection * r);
  static void remove_measure_request(const MeasuresCollection * r);

  static bool adjust_roi(const cv::Rect & src_roi, const cv::Size & image_size, cv::Rect * dst_roi);

Q_SIGNALS:
  void measurementsChanged();
  void framesMeasured(const QList<MeasuredFrame> & frames);

protected:
  QMeasureProvider(QObject * parent = nullptr);

protected:
  static QStringList _requested_channels;
  static MeasureRequest _requested_measurements;
  static std::vector<QMeasure*> _available_measures;
  static QMeasure* _valueMeasure;
};

#endif /* __QMeasureProvider_h__ */
