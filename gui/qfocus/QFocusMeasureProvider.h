/*
 * QFocusMeasureProvider.h
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFocusMeasureProvider_h__
#define __QFocusMeasureProvider_h__

#include <QtCore/QtCore>
#include <core/io/debayer.h>
#include <core/proc/focus.h>

class QFocusMeasureProvider:
    public QObject
{
  Q_OBJECT;
public:
  typedef QFocusMeasureProvider ThisClass;
  typedef QObject Base;

  enum {
    MAX_CHANNELS = 4
  };

  QFocusMeasureProvider(QObject * parent = nullptr );

  c_camera_focus_measure & measure();
  const c_camera_focus_measure & measure() const;

  int maxMeasurements() const;

  const QVector<double> & measurements(int channel) const;

  enum COLORID colorid() const;

  int bpp() const;

  QMutex & mutex();

  void load_parameters();
  void save_parameters();

  bool enabled() const;

public Q_SLOTS:
  virtual void setEnabled(bool v);

Q_SIGNALS:
  void dataChanged();

protected:
  bool getImageROI(cv::Mat * dst, const cv::Mat & frame,
      const QRect & rect,
      bool make_copy = true) const;

protected:
  QString sprefix_;
  c_camera_focus_measure measure_;
  QMutex mutex_;
  QVector<double> measurements_[MAX_CHANNELS];
  int max_measurements_ = 100;
  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpp_ = 0;
  int cn_ = 0;
  bool enabled_ = true;
};

#endif /* __QFocusMeasureProvider_h__ */
