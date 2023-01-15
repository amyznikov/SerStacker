/*
 * QImagingCamera.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImagingCamera__h__
#define __QImagingCamera__h__

#include <QtCore/QtCore>
#include "QCameraFrame.h"
#include <shared_mutex>

namespace serimager {

class QImagingCamera:
    public QObject
{
  Q_OBJECT;
public:
  typedef QImagingCamera ThisClass;
  typedef QObject Base;
  typedef std::shared_ptr<ThisClass> sptr;
  typedef std::unique_lock<std::shared_mutex> unique_lock;
  typedef std::shared_lock<std::shared_mutex> shared_lock;

  enum State {
    State_disconnected,
    State_connecting,
    State_connected,
    State_starting,
    State_started,
    State_stopping,
    State_disconnecting,
  };

  enum ExposureStatus
  {
    Exposure_idle = 0, // idle states, you can start exposure now
    Exposure_working, // exposing
    Exposure_success, // exposure finished and waiting for download
    Exposure_failed, //exposure failed, you need to start exposure again
  };

  QImagingCamera(QObject * parent = nullptr);

  virtual QString display_name() const = 0;
  virtual bool is_same_camera(const QImagingCamera::sptr & rhs) const = 0;
  virtual int drops() const = 0;

  State state() const;
  const QString & reason() const;

  bool connect();
  void disconnect();
  bool start();
  void stop();

  std::shared_mutex & mutex()
  {
    return mtx_;
  }

  std::condition_variable_any & condvar()
  {
    return condvar_;
  }

  const std::deque<QCameraFrame::sptr> & deque() const
  {
    return deque_;
  }

Q_SIGNALS:
  void stateChanged(QImagingCamera::State oldSate, QImagingCamera::State newState);
  void exposureStatusUpdate(QImagingCamera::ExposureStatus status, double exposure, double elapsed);
  void frameReceived();

protected:
  virtual bool device_is_connected() const = 0;
  virtual bool device_connect() = 0;
  virtual void device_disconnect() = 0;
  virtual bool device_start() = 0;
  virtual void device_stop() = 0;
  virtual int device_max_qsize() = 0;
  virtual void device_release_frame(const QCameraFrame::sptr & queue) = 0;
  virtual QCameraFrame::sptr device_recv_frame() = 0;

protected:
  void setState(State newState, const QString & reason = QString());
  virtual void onStateCanged(QImagingCamera::State oldSate, QImagingCamera::State newState);

protected:
  std::shared_mutex mtx_;
  std::condition_variable_any condvar_;
  std::deque<QCameraFrame::sptr> deque_;

  State current_state_ = State_disconnected;
  QString stateChangeReason_;
};

std::string fourccToString (uint32_t fourcc);
QString fourccToQString (uint32_t fourcc);


} /* namespace qserimager */

Q_DECLARE_METATYPE(serimager::QImagingCamera::sptr);
Q_DECLARE_METATYPE(serimager::QImagingCamera::State);

#endif /* __QImagingCamera__h__ */
