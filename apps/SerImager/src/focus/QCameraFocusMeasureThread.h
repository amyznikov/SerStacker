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
#include <core/proc/focus.h>

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

  c_local_contrast_measure & measure();
  const c_local_contrast_measure & measure() const;

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
  int max_measurements_ = 100;
  bool isEnabled_ = false;

  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpp_ = 0;

  c_local_contrast_measure measure_;


};

} /* namespace serimager */

#endif /* __QCameraFocusMeasureThread_h__ */
