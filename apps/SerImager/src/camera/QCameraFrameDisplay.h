/*
 * QCameraFrameDisplay.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraFrameDisplay_h__
#define __QCameraFrameDisplay_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qimageview/QMtfImageDisplayFunction.h>
#include "QImagingCamera.h"

namespace serimager {

class QCameraFrameDisplaySettings :
    public QMtfImageDisplaySettings
{
  Q_OBJECT;
public:
  typedef QCameraFrameDisplaySettings ThisClass;
  typedef QMtfImageDisplaySettings Base;

  QCameraFrameDisplaySettings(QObject * parent = nullptr);
  const c_enum_member * displayTypes() const override;

  void loadParameters() override;
  void saveParameters() const override;
};


class QCameraFrameDisplay:
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QCameraFrameDisplay ThisClass;
  typedef QImageEditor Base;
  typedef std::lock_guard<std::mutex> c_guard_lock;
  typedef std::unique_lock<std::mutex> c_unique_lock;

  QCameraFrameDisplay(QWidget * parent = nullptr);
  ~QCameraFrameDisplay();

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr & camera() const;

  const QCameraFrameDisplaySettings * displaySettings() const;
  QCameraFrameDisplaySettings * displaySettings();


Q_SIGNALS:
  void pixmapChanged();

protected Q_SLOTS:
  void onPixmapChanged();
  void onCameraStateChanged(QImagingCamera::State oldSate,
      QImagingCamera::State newState);

protected:
  void showCurrentDisplayImage() override;

protected:
  void startWorkerThread();
  void stopWorkerThread();
  void workerThread();
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QImagingCamera::sptr camera_;
  QCameraFrameDisplaySettings displaySettings_;
  QMtfImageDisplayFunction displayFunction_;

  std::mutex mutex_;
  enum WorkerState {
    Worker_Idle,
    Worker_Starting,
    Worker_Running,
    Worker_Stopping,
  } workerState_ = Worker_Idle;

  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpp_ = 0;
  QPixmap pixmap_;

};

} /* namespace qserimager */

#endif /* __QCameraFrameDisplay_h__ */
