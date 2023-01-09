/*
 * QCameraFocusMeasureThread.h
 *
 *  Created on: Jan 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraFocusMeasureThread_h__
#define __QCameraFocusMeasureThread_h__

#include <QtCore/QtCore>
#include "QImagingCamera.h"

namespace serimager {

class QCameraFocusMeasureThread:
    public QThread
{
  Q_OBJECT;
public:
  typedef QCameraFocusMeasureThread ThisClass;
  typedef QThread Base;

  enum {
    MAX_CHANNELS = 4
  };

  QCameraFocusMeasureThread(QObject * parent = nullptr);
  ~QCameraFocusMeasureThread();

  bool enabled() const;

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr camera() const;

  void setEps(double v);
  double eps() const;

  void setRoi(const QRect & roi);
  const QRect & roi() const;

  int maxMeasurements() const;

  const QVector<double> & measurements(int channel) const;

  enum COLORID colorid() const;

  int bpp() const;

  QMutex & mutex();


public Q_SLOTS:
  void setEnabled(bool v);

Q_SIGNALS:
  void dataChanged();

protected Q_SLOTS:
  void onCameraStateChanged();

protected :
  void run() override;

protected:
  QImagingCamera::sptr camera_;
  QMutex mutex_;
  QVector<double> measurements_[MAX_CHANNELS];
  QRect roi_;
  int max_measurements_ = 100;
  bool isEnabled_ = false;

  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpp_ = 0;
  double eps_ = 0;


};

} /* namespace serimager */

#endif /* __QCameraFocusMeasureThread_h__ */
