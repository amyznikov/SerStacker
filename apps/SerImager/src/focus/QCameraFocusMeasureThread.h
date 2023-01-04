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

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr camera() const;


  int maxDataLength() const
  {
    return maxDataSize_;
  }

  const QVector<double> & data(int channel) const
  {
    return data_[channel];
  }

  enum COLORID colorid() const
  {
    return colorid_;
  }

  int bpp() const
  {
    return bpp_;
  }


  QMutex & mutex()
  {
    return mutex_;
  }

  bool enabled() const;

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
  QVector<double> data_[MAX_CHANNELS];
  int maxDataSize_ = 100;
  bool isEnabled_ = false;

  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpp_ = 0;


};

} /* namespace serimager */

#endif /* __QCameraFocusMeasureThread_h__ */
