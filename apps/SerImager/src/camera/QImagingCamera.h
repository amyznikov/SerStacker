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
  typedef std::function<bool ()> PreStartProc;
  typedef std::unique_lock<std::shared_mutex> unique_lock;
  typedef std::shared_lock<std::shared_mutex> shared_lock;

  struct temporary_unlock
  {
    unique_lock & lk;
    temporary_unlock(unique_lock & _lk) : lk(_lk) { lk.unlock(); }
    ~temporary_unlock() { lk.lock(); }
  };

  enum State
  {
    State_disconnected,
    State_connecting,
    State_connected,
    State_starting,
    State_started,
    State_stop,
    State_disconnect,
  };

  enum ExposureStatus
  {
    Exposure_idle = 0, // idle state, you can start exposure now
    Exposure_working, // exposing
    Exposure_success, // exposure finished and waiting for download
    Exposure_failed, //exposure failed, you need to start exposure again
  };

  QImagingCamera(QObject * parent = nullptr);
  ~QImagingCamera();

  virtual QString display_name() const = 0;
  virtual QString parameters() const;

  virtual bool is_same_camera(const QImagingCamera::sptr & rhs) const = 0;
  virtual int drops() const = 0;

  State state() const;
  const QString & reason() const;

  const QRect & roi() const;
  void setRoi(const QRect & roi);

  bool connect();
  void disconnect();
  bool start();
  void stop();

  void addprestartproc(const PreStartProc * proc);
  void removeprestartproc(const PreStartProc * proc);

  std::shared_mutex & mutex();
  std::condition_variable_any & condvar();
  const std::deque<QCameraFrame::sptr> & deque() const;

Q_SIGNALS:
  void stateChanged(QImagingCamera::State oldState, QImagingCamera::State newState);
  void exposureStatusUpdate(QImagingCamera::ExposureStatus status, double exposure, double elapsed);
  void frameReceived();
  void roiChanged(const QRect & );

protected:
  virtual bool device_is_connected() const = 0;
  virtual bool device_connect() = 0;
  virtual void device_disconnect() = 0;
  virtual bool device_start() = 0;
  virtual void device_stop() = 0;
  virtual int device_max_qsize() = 0;
  virtual void device_release_frame(const QCameraFrame::sptr & frame) = 0;
  virtual bool device_recv_frame(QCameraFrame::sptr & fo) = 0;

protected:
  void finish();
  void setState(State newState, const QString & reason = QString());
  virtual void onStateCanged(QImagingCamera::State oldSate, QImagingCamera::State newState);

protected:
  std::shared_mutex _mtx;
  std::condition_variable_any _condvar;
  std::deque<QCameraFrame::sptr> _deque;
  std::vector<const PreStartProc*> _prestartproc;

  volatile State _current_state = State_disconnected;
  QString _stateChangeReason;

  QRect _roi;
};

std::string fourccToString (uint32_t fourcc);
QString fourccToQString (uint32_t fourcc);


} /* namespace serimager */

Q_DECLARE_METATYPE(serimager::QImagingCamera::sptr);
Q_DECLARE_METATYPE(serimager::QImagingCamera::State);

#endif /* __QImagingCamera__h__ */
