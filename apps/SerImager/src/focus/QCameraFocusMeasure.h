/*
 * QCameraFocusMeasure.h
 *
 *  Created on: Jan 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraFocusMeasure_h__
#define __QCameraFocusMeasure_h__

#include "QImagingCamera.h"
#include <gui/qfocus/QFocusMeasureProvider.h>

namespace serimager {

class QCameraFocusMeasure:
    public QFocusMeasureProvider
{
  Q_OBJECT;
public:
  typedef QCameraFocusMeasure ThisClass;
  typedef QFocusMeasureProvider Base;

  QCameraFocusMeasure(QObject * parent = nullptr);
  ~QCameraFocusMeasure();

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr camera() const;

public Q_SLOTS:
  void setEnabled(bool v) override;
  void onCameraStateChanged();

protected:

  class CameraMonitorThread :
      public QThread
  {
  public:
    typedef CameraMonitorThread ThisClass;
    typedef QThread Base;

    CameraMonitorThread(QCameraFocusMeasure * parent);

  protected :
    void run() override;

  protected :
    QCameraFocusMeasure * p_;
  };

  QImagingCamera::sptr camera_;
  CameraMonitorThread thread_;
};

} /* namespace serimager */

#endif /* __QCameraFocusMeasureThread_h__ */
